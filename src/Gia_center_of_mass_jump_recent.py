
# imports
try:
  import rospy
  import tf
  ros_enabled = True
except:
  print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
  ros_enabled = False


import os, sys
import numpy as np


from pushpull_suctioncup_106a.msg import PushPull
from suction_cup.srv import Enable


from helperFunction.rtde_helper import rtdeHelp
from helperFunction.SuctionP_callback_helper import P_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp




def _sample_four_pressure_avg(P_help, n_samples=10, dt=0.012):
  """Mean of last n_samples readings from four_pressure (4 channels)."""
  vals = []
  for _ in range(n_samples):
    v = np.asarray(P_help.four_pressure, dtype=float).ravel()
    if v.size < 4:
      v = np.pad(v, (0, 4 - int(v.size)))
    vals.append(v[:4].copy())
    rospy.sleep(dt)
  return np.mean(np.stack(vals, axis=0), axis=0)


def _go_delta_xyz(rtde_help, dx, dy, dz, speed, acc):
  cur = rtde_help.getCurrentPose()
  p = [
    cur.pose.position.x + dx,
    cur.pose.position.y + dy,
    cur.pose.position.z + dz,
  ]
  o = [
    cur.pose.orientation.x,
    cur.pose.orientation.y,
    cur.pose.orientation.z,
    cur.pose.orientation.w,
  ]
  rtde_help.goToPose(rtde_help.getPoseObj(p, o), speed=speed, acc=acc)


def main(args):


  np.set_printoptions(precision=4)


  # controller node
  rospy.init_node('edg_experiment')


  # Setup helper functions
  rtde_help = rtdeHelp(125)

  P_help = P_CallbackHelp()  # Pressure sensor helper
  rospy.sleep(0.5)
  
  # === File saving helper ===
  file_help = fileSaveHelp()
  
  # === Data logging service ===
  rospy.wait_for_service('data_logging')
  dataLoggerEnable = rospy.ServiceProxy('data_logging', Enable)
  dataLoggerEnable(False)
  rospy.sleep(1)
  file_help.clearTmpFolder()
 
  # === PushPull 토픽/메세지 ===
  DUTYCYCLE_100 = 100
  DUTYCYCLE_0 = 0
  PULL_STATE = 1
  PUSH_STATE = 2
  OFF_STATE = 0
  PushPull_pub = rospy.Publisher('PushPull', PushPull, queue_size=10)
  rospy.sleep(0.5)
  msg = PushPull()


  # Set the TCP offset and calibration matrix (ex, suction cup: 0.150, ATI_default: 0.464)
  # You can set the TCP offset here, but it is recommended to set it in the UR program.
  # If you set it here, endEffectorPose will be different from the actual pose.
  # rtde_help.setTCPoffset([0, 0, 0.464, 0, 0, 0])
  # rospy.sleep(0.2)


  # Set the pose A

  # positionA = [0.50684, -0.02715, 0.01155]  # for paper
  # positionA = [0.50777, -0.00597, 0.07356]  # for toy
  # positionA = [0.50800, 0.05516, 0.04611]  # for glue
  # positionA = [0.51277, 0.04676, 0.02826]  # for acrylic plate
  # positionA = [0.51572, 0.06649, 0.01193]  # for pcb
  # positionA = [0.53469, -0.02315, 0.17156]  # for round_jar
  positionA = [0.50672, 0.01411, 0.01377]  # for baby_toy




  # Calculate positionA_true by adding xoffset (mm) to y coordinate (second element) in meters
  xoffset_m = getattr(args, "xoffset", 0) * 1e-3  # Convert mm to meters
  positionA_true = [positionA[0]-xoffset_m, positionA[1] , positionA[2]]

  positionA_y_end = (0.01299 - 0.08)  # Target y position (10cm from start: 0.02501 + 0.08 = 0.10501)
  orientationA = tf.transformations.quaternion_from_euler(np.pi, 0, -np.pi/2,'sxyz') #static (s) rotating (r)
  poseA = rtde_help.getPoseObj(positionA_true, orientationA)




  # try block so that we can have a keyboard exception
  try:


    if not getattr(args, "skip_pose_enter", False):
      input("Press <Enter> to go to pose A (positionA_true)")
    rtde_help.goToPose(poseA)
    rospy.sleep(1)
    print("poseA: ", rtde_help.getCurrentPose())

    # --- Haptic center-of-mass search (4ch pressure, Z-probe gradient) ---
    probe_z_mm = float(getattr(args, "haptic_probe_z_mm", 10.0))
    push_hold_above_mm = float(getattr(args, "haptic_push_hold_above_mm", 3.0))
    step_xy_mm = float(getattr(args, "haptic_step_xy_mm", 10.0))
    final_lift_mm = float(getattr(args, "haptic_final_lift_mm", 50.0))
    flat_ptp = float(getattr(args, "haptic_flat_weight_ptp", 0.11))
    z_fast_speed = float(getattr(args, "haptic_z_probe_speed", 1.5))
    z_fast_acc = float(getattr(args, "haptic_z_probe_acc", 1.5))
    xy_speed = float(getattr(args, "haptic_xy_speed", 0.12))
    xy_acc = float(getattr(args, "haptic_xy_acc", 0.12))
    max_iters = int(getattr(args, "haptic_max_iters", 80))
    min_lat_m = float(getattr(args, "haptic_min_lateral_mm", 0.25)) * 1e-3
    min_bal_L1 = float(getattr(args, "haptic_min_balanced_l1", 25.0))

    probe_z_m = probe_z_mm * 1e-3
    hold_above_m = push_hold_above_mm * 1e-3
    step_xy_m = step_xy_mm * 1e-3
    final_lift_m = final_lift_mm * 1e-3

    # One row per step; columns defined in STEP_COLS (saved with the mat).
    STEP_COLS = [
      "step",
      "t_sec",
      "x_m", "y_m", "z_m",
      "p_before_ch0", "p_before_ch1", "p_before_ch2", "p_before_ch3",
      "p_top_ch0", "p_top_ch1", "p_top_ch2", "p_top_ch3",
      "grad_ch0", "grad_ch1", "grad_ch2", "grad_ch3",
      "grad_bal_ch0", "grad_bal_ch1", "grad_bal_ch2", "grad_bal_ch3",
      "w_ch0", "w_ch1", "w_ch2", "w_ch3",
      "dx_mm", "dy_mm",
      "sum_abs_grad_bal",
      "max_w_minus_min_w",
      "abs_dxy_mm",
      "xy_moved",
      "stop_weak_bal",
      "stop_flat_w",
      "stop_tiny_dxy",
    ]
    data_step_rows = []
    timestamp_start_time = rospy.Time.now().to_sec()

    P_help.startSampling()
    rospy.sleep(0.5)
    P_help.setNowAsOffset()
    rospy.sleep(0.5)

    dataLoggerEnable(True)
    rospy.sleep(0.2)

    print(
      "COM haptic: grad from p_top-p_before (Z apex vs start). "
      "After apex: descend to startZ+%.1fmm (min of request & probe-0.5mm), PUSH 1s, OFF 1s, then start Z. "
      "grad_bal=grad-mean(grad); w from |grad_bal|; dx=s*(w0-w2), dy=s*(w1-w3)."
      % (push_hold_above_mm,)
    )
    print(
      "Stop if sum|grad_bal| < %.1f OR max(w)-min(w) < %.3f OR |dxy| tiny."
      % (min_bal_L1, flat_ptp)
    )
    print("COM haptic: publishing PULL_STATE (suction on for probe); starting Z-probe loop.")
    msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
    PushPull_pub.publish(msg)
    rospy.sleep(0.2)

    def _append_step_row(
      step_i, ts, pos, p_before, p_top, grad, grad_bal, w,
      dx_mm, dy_mm, s_bal, w_ptp, abs_dxy_mm, xy_moved,
      stop_weak_bal, stop_flat_w, stop_tiny_dxy,
    ):
      data_step_rows.append([
        float(step_i),
        float(ts),
        float(pos[0]), float(pos[1]), float(pos[2]),
        float(p_before[0]), float(p_before[1]), float(p_before[2]), float(p_before[3]),
        float(p_top[0]), float(p_top[1]), float(p_top[2]), float(p_top[3]),
        float(grad[0]), float(grad[1]), float(grad[2]), float(grad[3]),
        float(grad_bal[0]), float(grad_bal[1]), float(grad_bal[2]), float(grad_bal[3]),
        float(w[0]), float(w[1]), float(w[2]), float(w[3]),
        float(dx_mm), float(dy_mm),
        float(s_bal),
        float(w_ptp),
        float(abs_dxy_mm),
        float(xy_moved),
        float(stop_weak_bal),
        float(stop_flat_w),
        float(stop_tiny_dxy),
      ])

    def _save_com_mat(append_suffix=""):
      dataLoggerEnable(False)
      rospy.sleep(0.1)
      table = np.asarray(data_step_rows, dtype=float)
      if table.size == 0:
        table = np.zeros((0, len(STEP_COLS)), dtype=float)
      args.data_com_step_table = table
      args.data_com_step_columns = np.array(STEP_COLS, dtype=object)
      # Convenience views (same rows as table; no data loss if analysis used old keys)
      if table.shape[0] > 0:
        args.data_com_iteration = table[:, 0]
        args.data_com_timestamps = table[:, 1]
        args.data_com_positions = table[:, 2:5]
        args.data_com_pressure_before = table[:, 5:9]
        args.data_com_pressure_top = table[:, 9:13]
        args.data_com_gradients = table[:, 13:17]
        args.data_com_gradients_balanced = table[:, 17:21]
        args.data_com_weights = table[:, 21:25]
        args.data_com_xy_steps_mm = table[:, 25:27]
        args.data_com_sum_abs_grad_bal = table[:, 27]
        args.data_com_max_w_minus_min_w = table[:, 28]
        args.data_com_abs_dxy_mm = table[:, 29]
        args.data_com_xy_moved = table[:, 30]
      else:
        args.data_com_iteration = np.zeros((0,), dtype=float)
        args.data_com_timestamps = np.zeros((0,), dtype=float)
        args.data_com_positions = np.zeros((0, 3), dtype=float)
        args.data_com_pressure_before = np.zeros((0, 4), dtype=float)
        args.data_com_pressure_top = np.zeros((0, 4), dtype=float)
        args.data_com_gradients = np.zeros((0, 4), dtype=float)
        args.data_com_gradients_balanced = np.zeros((0, 4), dtype=float)
        args.data_com_weights = np.zeros((0, 4), dtype=float)
        args.data_com_xy_steps_mm = np.zeros((0, 2), dtype=float)
        args.data_com_sum_abs_grad_bal = np.zeros((0,), dtype=float)
        args.data_com_max_w_minus_min_w = np.zeros((0,), dtype=float)
        args.data_com_abs_dxy_mm = np.zeros((0,), dtype=float)
        args.data_com_xy_moved = np.zeros((0,), dtype=float)
      args.positionA = positionA
      args.positionA_true = positionA_true
      cur_done = rtde_help.getCurrentPose()
      args.com_final_pose_xyz = np.array(
        [cur_done.pose.position.x, cur_done.pose.position.y, cur_done.pose.position.z],
        dtype=float,
      )
      xoff = getattr(args, "xoffset", 0)
      tag = "COM_haptic_material_%s_xoffset_%s%s" % (args.material, xoff, append_suffix)
      file_help.saveDataParams(args, appendTxt=tag)
      file_help.clearTmpFolder()
      print(
        "Saved COM step table: %d steps x %d cols -> data_com_step_table / data_com_step_columns"
        % (table.shape[0], table.shape[1])
      )

    it = 0
    while it < max_iters and not rospy.is_shutdown():
      it += 1
      cur = rtde_help.getCurrentPose()
      pos = [cur.pose.position.x, cur.pose.position.y, cur.pose.position.z]

      msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
      PushPull_pub.publish(msg)
      rospy.sleep(0.05)

      p_before = _sample_four_pressure_avg(P_help)

      # Fast +Z probe (base +Z), sample p_top at apex
      _go_delta_xyz(rtde_help, 0.0, 0.0, probe_z_m, z_fast_speed, z_fast_acc)
      rospy.sleep(0.02)
      p_top = _sample_four_pressure_avg(P_help)

      # Descend to (start Z + hold_above), not full contact: PUSH 1s, OFF 1s, then to start Z
      effective_hold_m = min(hold_above_m, max(1e-4, probe_z_m - 0.5e-3))
      dz_to_hold = effective_hold_m - probe_z_m
      _go_delta_xyz(rtde_help, 0.0, 0.0, dz_to_hold, z_fast_speed, z_fast_acc)
      rospy.sleep(0.02)

      msg.state, msg.pwm = PUSH_STATE, DUTYCYCLE_100
      PushPull_pub.publish(msg)
      rospy.sleep(1.0)
      msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
      PushPull_pub.publish(msg)
      rospy.sleep(1.0)

      _go_delta_xyz(rtde_help, 0.0, 0.0, -effective_hold_m, z_fast_speed, z_fast_acc)
      rospy.sleep(0.02)

      grad = p_top - p_before
      grad_bal = grad - np.mean(grad)
      s_bal = float(np.sum(np.abs(grad_bal)))

      if s_bal < 1e-12:
        w = np.ones(4, dtype=float) / 4.0
      else:
        w = np.abs(grad_bal) / s_bal

      # Lateral from weights: dx = step*(w0-w2), dy = step*(w1-w3)
      # (ch1 +x, ch2 +y, ch3 -x, ch4 -y in 1-based numbering -> w[0]..w[3])
      dx_m = step_xy_m * (float(w[0]) - float(w[2]))
      dy_m = step_xy_m * (float(w[1]) - float(w[3]))
      lat_mag = float(np.hypot(dx_m, dy_m))
      dx_mm = dx_m * 1e3
      dy_mm = dy_m * 1e3
      abs_dxy_mm = lat_mag * 1e3
      w_ptp = float(np.max(w) - np.min(w))
      ts = rospy.Time.now().to_sec() - timestamp_start_time

      print(
        "[COM %d] sum|grad_bal|=%.2f grad=%s grad_bal=%s w=%s dxy_mm=(%.3f,%.3f) w_ptp=%.4f"
        % (
          it,
          s_bal,
          np.round(grad, 2),
          np.round(grad_bal, 2),
          np.round(w, 3),
          dx_mm,
          dy_mm,
          w_ptp,
        )
      )

      weak_bal = s_bal < min_bal_L1
      flat_weights = w_ptp < flat_ptp
      tiny_move = lat_mag < min_lat_m

      # No useful lateral update => hold XY, PULL, +Z final lift
      if weak_bal or flat_weights or tiny_move:
        _append_step_row(
          it, ts, pos, p_before, p_top, grad, grad_bal, w,
          dx_mm, dy_mm, s_bal, w_ptp, abs_dxy_mm, 0.0,
          float(weak_bal), float(flat_weights), float(tiny_move),
        )
        if weak_bal and flat_weights and tiny_move:
          why = "weak balanced + flat w + tiny |dxy|"
        elif weak_bal:
          why = "weak asymmetry sum|grad_bal| < %.1f (near COM / common-mode)" % min_bal_L1
        elif flat_weights:
          why = "flat w (max(w)-min(w) < %.4f)" % flat_ptp
        else:
          why = "tiny move (|dxy| < %.3f mm)" % (min_lat_m * 1e3)
        print("Converged (%s): hold XY, PULL, +Z %.1f mm" % (why, final_lift_mm))
        msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
        PushPull_pub.publish(msg)
        rospy.sleep(0.2)
        _go_delta_xyz(rtde_help, 0.0, 0.0, final_lift_m, 0.25, 0.25)
        rospy.sleep(0.4)
        _save_com_mat("_converged")
        msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
        PushPull_pub.publish(msg)
        rospy.sleep(0.1)
        P_help.stopSampling()
        print("============ COM haptic search complete (converged).")
        return

      _append_step_row(
        it, ts, pos, p_before, p_top, grad, grad_bal, w,
        dx_mm, dy_mm, s_bal, w_ptp, abs_dxy_mm, 1.0,
        0.0, 0.0, 0.0,
      )
      _go_delta_xyz(rtde_help, dx_m, dy_m, 0.0, xy_speed, xy_acc)
      rospy.sleep(0.05)

      msg.state, msg.pwm = PULL_STATE, DUTYCYCLE_100
      PushPull_pub.publish(msg)
      rospy.sleep(0.15)

    print("Max iterations (%d); saving and stopping." % max_iters)
    _save_com_mat("_max_iters")
    msg.state, msg.pwm = OFF_STATE, DUTYCYCLE_0
    PushPull_pub.publish(msg)
    rospy.sleep(0.1)
    P_help.stopSampling()
    print("============ COM haptic search stopped (max iters).")
    return

  except rospy.ROSInterruptException:
    dataLoggerEnable(False)
    return
  except KeyboardInterrupt:
    dataLoggerEnable(False)
    return


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--deformation', type=float, default=3, help='(unused in COM haptic mode) legacy arg')
  parser.add_argument('--material', type=str, default="paper", help='object to test')
  parser.add_argument('--xoffset', type=float, default=0, help='X offset (mm) subtracted from positionA x (see positionA_true)')
  parser.add_argument('--skip_pose_enter', action='store_true', help='Do not wait for Enter before move to pose A (use with care)')
  parser.add_argument('--haptic_probe_z_mm', type=float, default=10.0, help='Fast Z probe height (mm, +base Z)')
  parser.add_argument('--haptic_push_hold_above_mm', type=float, default=3.0, help='After p_top: stop this many mm above cycle start Z for PUSH/OFF (capped by probe height)')
  parser.add_argument('--haptic_z_probe_speed', type=float, default=1.5, help='RTDE moveL TCP linear speed for Z probe (m/s)')
  parser.add_argument('--haptic_z_probe_acc', type=float, default=1.5, help='RTDE moveL TCP linear accel for Z probe (m/s^2)')
  parser.add_argument('--haptic_step_xy_mm', type=float, default=10.0, help='Scale s (mm) in dx=s*(w0-w2), dy=s*(w1-w3); w from |grad_bal|/sum|grad_bal|')
  parser.add_argument('--haptic_final_lift_mm', type=float, default=50.0, help='Z lift after convergence (mm)')
  parser.add_argument('--haptic_flat_weight_ptp', type=float, default=0.11, help='Stop if max(w)-min(w) < this (w from balanced gradient)')
  parser.add_argument('--haptic_min_balanced_l1', type=float, default=25.0, help='Stop if sum|grad-mean(grad)| below this (pressure units); no asymmetry')
  parser.add_argument('--haptic_min_lateral_mm', type=float, default=0.25, help='Stop if hypot(dx,dy) below this (mm)')
  parser.add_argument('--haptic_max_iters', type=int, default=80, help='Safety cap on search iterations')

  cli_args = parser.parse_args()
  main(cli_args)
