#!/usr/bin/env roseus
;; scissor-robot-demo.l
;; Demonstration of scissor control integrated with robot manipulation

(load "scissor-interface.l")

;; Robot demo class that integrates scissor control
(defclass robot-scissor-demo
  :super object
  :slots (scissor-if
          demo-states))

(defmethod robot-scissor-demo
  (:init (&key (scissor-config nil))
    ;; Initialize scissor interface
    (if scissor-config
        (setq scissor-if (instance scissor-interface :init
                                  :joint-name (cdr (assoc :joint-name scissor-config))
                                  :open-position (cdr (assoc :open-position scissor-config))
                                  :close-position (cdr (assoc :close-position scissor-config))
                                  :open-is-max (cdr (assoc :open-is-max scissor-config))))
      (setq scissor-if (instance scissor-interface :init)))

    ;; Define demo states
    (setq demo-states '(:idle :approach :grasp :cut :release :retract))

    (ros::ros-info "Robot scissor demo initialized")
    self)

  ;; Get scissor interface
  (:get-scissor () scissor-if)

  ;; Simulation of robot approaching object
  (:approach-object ()
    (ros::ros-info "Robot approaching object...")
    (ros::sleep 1.0)
    (ros::ros-info "Robot in position"))

  ;; Simulation of robot grasping
  (:grasp-object ()
    (ros::ros-info "Robot grasping object...")
    (ros::sleep 0.5)
    (ros::ros-info "Object grasped"))

  ;; Perform cutting operation
  (:perform-cut (&optional (pre-cut-delay 0.5))
    (ros::ros-info "Preparing for cutting operation...")

    ;; Ensure scissor is open before cutting
    (send scissor-if :open 1.0)
    (ros::sleep (+ 1.0 pre-cut-delay))

    ;; Perform cutting motion
    (ros::ros-info "Performing cut...")
    (send scissor-if :close 0.8)
    (ros::sleep 1.0)

    ;; Open scissor after cut
    (send scissor-if :open 0.8)
    (ros::sleep 1.0)

    (ros::ros-info "Cutting operation completed"))

  ;; Release object
  (:release-object ()
    (ros::ros-info "Robot releasing object...")
    (ros::sleep 0.5)
    (ros::ros-info "Object released"))

  ;; Retract robot
  (:retract-robot ()
    (ros::ros-info "Robot retracting...")
    (ros::sleep 1.0)
    (ros::ros-info "Robot retracted to safe position"))

  ;; Full cutting sequence
  (:execute-cutting-sequence (&optional (num-objects 3))
    (ros::ros-info "=== Starting Cutting Sequence for ~A objects ===" num-objects)

    ;; Wait for scissor to be ready
    (send scissor-if :wait-for-joint-state 3.0)

    (dotimes (i num-objects)
      (ros::ros-info "Processing object ~A/~A" (1+ i) num-objects)

      ;; Execute full sequence
      (send self :approach-object)
      (send self :grasp-object)
      (send self :perform-cut 0.3)
      (send self :release-object)
      (send self :retract-robot)

      ;; Wait between objects
      (when (< i (1- num-objects))
        (ros::ros-info "Waiting before next object...")
        (ros::sleep 1.5)))

    (ros::ros-info "=== Cutting sequence completed ==="))

  ;; Safety check routine
  (:safety-check ()
    (ros::ros-info "Performing safety check...")

    ;; Check if scissor responds
    (let ((initial-pos (send scissor-if :get-current-position)))
      (ros::ros-info "Initial position: ~A rad" initial-pos)

      ;; Test open
      (send scissor-if :open 1.0)
      (ros::sleep 1.5)
      (let ((open-pos (send scissor-if :get-current-position)))
        (ros::ros-info "Open position: ~A rad" open-pos))

      ;; Test close
      (send scissor-if :close 1.0)
      (ros::sleep 1.5)
      (let ((close-pos (send scissor-if :get-current-position)))
        (ros::ros-info "Close position: ~A rad" close-pos))

      ;; Return to initial position
      (send scissor-if :move-to initial-pos 1.0)
      (ros::sleep 1.5)

      (ros::ros-info "Safety check completed")))

  ;; Emergency stop
  (:emergency-stop ()
    (ros::ros-warn "EMERGENCY STOP ACTIVATED")
    (send scissor-if :open 0.3)  ;; Quick open for safety
    (ros::ros-warn "Scissor opened for safety"))

  ;; Demo status
  (:print-demo-status ()
    (ros::ros-info "=== Robot Scissor Demo Status ===")
    (send scissor-if :print-status)
    (ros::ros-info "Demo states available: ~A" demo-states)
    (ros::ros-info "=============================="))
)

;; Demo functions

;; Demo 1: Basic robot cutting operation
(defun robot-demo-basic ()
  "Basic robot cutting demonstration"

  (ros::ros-info "=== Robot Demo 1: Basic Cutting Operation ===")

  ;; Create demo with standard configuration
  (setq *robot-demo* (instance robot-scissor-demo :init))

  ;; Perform safety check first
  (send *robot-demo* :safety-check)
  (ros::sleep 1)

  ;; Execute single cutting operation
  (send *robot-demo* :execute-cutting-sequence 1)

  (ros::ros-info "Basic demo completed\n"))

;; Demo 2: Multiple object processing
(defun robot-demo-multiple ()
  "Multiple object processing demonstration"

  (ros::ros-info "=== Robot Demo 2: Multiple Object Processing ===")

  (unless (boundp '*robot-demo*)
    (setq *robot-demo* (instance robot-scissor-demo :init)))

  ;; Process multiple objects
  (send *robot-demo* :execute-cutting-sequence 3)

  (ros::ros-info "Multiple object demo completed\n"))

;; Demo 3: Different hardware configuration
(defun robot-demo-inverted-hardware ()
  "Demo with inverted hardware configuration"

  (ros::ros-info "=== Robot Demo 3: Inverted Hardware Configuration ===")

  ;; Create demo with inverted configuration
  (setq inverted-config '((:joint-name . "sample_joint")
                         (:open-position . -3.14)
                         (:close-position . 0.5)
                         (:open-is-max . nil)))

  (setq *robot-demo-inv* (instance robot-scissor-demo :init
                                  :scissor-config inverted-config))

  ;; Show configuration
  (send *robot-demo-inv* :print-demo-status)

  ;; Perform cutting operation
  (send *robot-demo-inv* :execute-cutting-sequence 2)

  (ros::ros-info "Inverted hardware demo completed\n"))

;; Main demo function
(defun scissor-robot-demos ()
  "Run all robot integration demos"

  (ros::ros-info "Starting robot scissor integration demos...")

  ;; Run all demos
  (robot-demo-basic)
  (ros::sleep 2)

  (robot-demo-multiple)
  (ros::sleep 2)

  (robot-demo-inverted-hardware)

  (ros::ros-info "All robot demos completed!"))

;; Interactive demo function
(defun interactive-scissor-demo ()
  "Interactive demonstration with user input"

  (ros::ros-info "=== Interactive Scissor Demo ===")
  (ros::ros-info "This demo allows you to control the scissor interactively")

  (setq *interactive-demo* (instance robot-scissor-demo :init))
  (send *interactive-demo* :safety-check)

  (while t
    (format t "~%Choose an action:~%")
    (format t "1. Open scissor~%")
    (format t "2. Close scissor~%")
    (format t "3. Toggle scissor~%")
    (format t "4. Perform cut~%")
    (format t "5. Execute cutting sequence~%")
    (format t "6. Safety check~%")
    (format t "7. Print status~%")
    (format t "8. Emergency stop~%")
    (format t "9. Exit~%")
    (format t "Enter choice (1-9): ")

    (let ((choice (read)))
      (case choice
        (1 (send (send *interactive-demo* :get-scissor) :open))
        (2 (send (send *interactive-demo* :get-scissor) :close))
        (3 (send (send *interactive-demo* :get-scissor) :toggle))
        (4 (send *interactive-demo* :perform-cut))
        (5 (progn
             (format t "Enter number of objects: ")
             (let ((num (read)))
               (send *interactive-demo* :execute-cutting-sequence num))))
        (6 (send *interactive-demo* :safety-check))
        (7 (send *interactive-demo* :print-demo-status))
        (8 (send *interactive-demo* :emergency-stop))
        (9 (return))
        (t (format t "Invalid choice. Please enter 1-9.~%")))))

  (ros::ros-info "Interactive demo ended"))

;; Run the demos if this file is executed directly
(when (string= (car (last lisp::*eustop-argument*)) "scissor-robot-demo.l")
  (scissor-robot-demos))