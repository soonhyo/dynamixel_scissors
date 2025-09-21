#!/usr/bin/env roseus
;; scissor-interface.l
;; EusLisp interface for Dynamixel Scissor Control

(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "trajectory_msgs")
(ros::load-ros-manifest "sensor_msgs")

;; Scissor Control Interface Class
(defclass scissor-interface
  :super object
  :slots (trajectory-pub
          joint-sub
          current-position
          target-joint
          goal-id-counter
          config))

(defmethod scissor-interface
  (:init (&key (joint-name "sample_joint")
               (trajectory-topic "/sample_robot/position_joint_trajectory_controller/follow_joint_trajectory/goal")
               (joint-states-topic "/sample_robot/joint_states")
               (open-position 0.5)
               (close-position -3.14)
               (open-is-max t))
    ;; Initialize ROS node
    (ros::roseus "scissor_euslisp_interface")

    ;; Store configuration
    (setq config (list (cons :joint-name joint-name)
                       (cons :open-position open-position)
                       (cons :close-position close-position)
                       (cons :open-is-max open-is-max)))

    ;; Initialize variables
    (setq target-joint joint-name)
    (setq current-position 0.0)
    (setq goal-id-counter 0)

    ;; Initialize publishers and subscribers
    (ros::advertise trajectory-topic
                         control_msgs::FollowJointTrajectoryActionGoal)
    (setq trajectory-pub
          trajectory-topic)

    (ros::subscribe joint-states-topic
                         sensor_msgs::JointState
                         #'send self :joint-state-callback)

    (setq joint-sub joint-states-topic)

    (ros::ros-info "Scissor interface initialized")
    (ros::ros-info "Joint name: ~A" target-joint)
    (ros::ros-info "Open position: ~A rad" open-position)
    (ros::ros-info "Close position: ~A rad" close-position)
    (ros::ros-info "Open is max: ~A" open-is-max)
    self)

  ;; Joint state callback
  (:joint-state-callback (msg)
    (let ((joint-names (send msg :name))
          (positions (send msg :position)))
      (dotimes (i (length joint-names))
        (when (string= (elt joint-names i) target-joint)
          (setq current-position (elt positions i))
          (return)))))

  ;; Get current position
  (:get-current-position ()
    current-position)

  ;; Get open position from config
  (:get-open-position ()
    (cdr (assoc :open-position config)))

  ;; Get close position from config
  (:get-close-position ()
    (cdr (assoc :close-position config)))

  ;; Check if open is max
  (:is-open-max ()
    (cdr (assoc :open-is-max config)))

  ;; Publish trajectory goal
  (:publish-trajectory (target-position &optional (duration 1.0))
    (incf goal-id-counter)
    (let ((goal-msg (instance control_msgs::FollowJointTrajectoryActionGoal :init))
          (trajectory-point (instance trajectory_msgs::JointTrajectoryPoint :init))
          (goal-id (instance actionlib_msgs::GoalID :init)))

      ;; Set header
      (send goal-msg :header :stamp (ros::time-now))
      (send goal-msg :header :frame_id "")

      ;; Set goal ID
      (send goal-id :stamp (ros::time-now))
      (send goal-id :id (format nil "scissor_goal_~A" goal-id-counter))
      (send goal-msg :goal_id goal-id)

      ;; Set trajectory header
      (send goal-msg :goal :trajectory :header :stamp (ros::time-now))
      (send goal-msg :goal :trajectory :header :frame_id "")

      ;; Set joint names
      (send goal-msg :goal :trajectory :joint_names (list target-joint))

      ;; Create trajectory point
      (send trajectory-point :positions (list target-position))
      (send trajectory-point :velocities (list 0.0))
      (send trajectory-point :accelerations (list 0.0))
      (send trajectory-point :effort (list 0.0))
      (send trajectory-point :time_from_start (ros::time duration))

      ;; Set trajectory points
      (send goal-msg :goal :trajectory :points (list trajectory-point))

      ;; Set tolerances (empty for default)
      (send goal-msg :goal :path_tolerance nil)
      (send goal-msg :goal :goal_tolerance nil)
      (send goal-msg :goal :goal_time_tolerance (ros::time 0))

      ;; Publish
      (ros::publish trajectory-pub goal-msg)
      (ros::ros-info "Published trajectory to position: ~A rad" target-position)
      t))

  ;; Open scissor
  (:open (&optional (duration 1.0))
    (let ((open-pos (send self :get-open-position)))
      (send self :publish-trajectory open-pos duration)
      (ros::ros-info "Opening scissor to ~A rad" open-pos)))

  ;; Close scissor
  (:close (&optional (duration 1.0))
    (let ((close-pos (send self :get-close-position)))
      (send self :publish-trajectory close-pos duration)
      (ros::ros-info "Closing scissor to ~A rad" close-pos)))

  ;; Move to specific position
  (:move-to (position &optional (duration 1.0))
    (send self :publish-trajectory position duration)
    (ros::ros-info "Moving scissor to ~A rad" position))

  ;; Toggle scissor state
  (:toggle (&optional (duration 1.0))
    (let ((open-pos (send self :get-open-position))
          (close-pos (send self :get-close-position))
          (current-pos (send self :get-current-position)))
      (let ((dist-to-open (abs (- current-pos open-pos)))
            (dist-to-close (abs (- current-pos close-pos))))
        (if (> dist-to-open dist-to-close)
            (send self :open duration)
          (send self :close duration)))))

  ;; Wait for joint state
  (:wait-for-joint-state (&optional (timeout 5.0))
    (let ((start-time (ros::time-now))
          (received nil))
      (while (and (not received)
                  (< (send (ros::time- (ros::time-now) start-time) :to-sec) timeout))
        (ros::spin-once)
        (when (not (= current-position 0.0))
          (setq received t))
        (ros::sleep 0.1))
      received))

  ;; Get status information
  (:get-status ()
    (list (cons :current-position current-position)
          (cons :open-position (send self :get-open-position))
          (cons :close-position (send self :get-close-position))
          (cons :joint-name target-joint)
          (cons :open-is-max (send self :is-open-max))))

  ;; Print status
  (:print-status ()
    (let ((status (send self :get-status)))
      (ros::ros-info "=== Scissor Status ===")
      (ros::ros-info "Current position: ~A rad" (cdr (assoc :current-position status)))
      (ros::ros-info "Open position: ~A rad" (cdr (assoc :open-position status)))
      (ros::ros-info "Close position: ~A rad" (cdr (assoc :close-position status)))
      (ros::ros-info "Joint name: ~A" (cdr (assoc :joint-name status)))
      (ros::ros-info "Open is max: ~A" (cdr (assoc :open-is-max status)))
      (ros::ros-info "===================")))
)