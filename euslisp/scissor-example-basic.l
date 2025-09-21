#!/usr/bin/env roseus
;; scissor-example-basic.l
;; Basic usage example for EusLisp scissor interface

(load "scissor-interface.l")

;; Basic usage example
(defun scissor-basic-example ()
  "Basic example showing open, close, and toggle operations"

  ;; Create scissor interface with default settings
  (setq *scissor* (instance scissor-interface :init))

  ;; Wait for initial joint state
  (ros::ros-info "Waiting for joint state...")
  (if (send *scissor* :wait-for-joint-state 5.0)
      (ros::ros-info "Joint state received successfully")
    (ros::ros-warn "No joint state received, continuing anyway..."))

  ;; Print initial status
  (send *scissor* :print-status)

  ;; Example 1: Open scissor
  (ros::ros-info "Example 1: Opening scissor...")
  (send *scissor* :open 2.0)  ;; 2 second duration
  (ros::sleep 3)

  ;; Example 2: Close scissor
  (ros::ros-info "Example 2: Closing scissor...")
  (send *scissor* :close 2.0)
  (ros::sleep 3)

  ;; Example 3: Toggle scissor (goes to farther position)
  (ros::ros-info "Example 3: Toggling scissor...")
  (send *scissor* :toggle 1.5)
  (ros::sleep 2)

  ;; Example 4: Move to specific position
  (ros::ros-info "Example 4: Moving to specific position...")
  (send *scissor* :move-to -1.0 1.0)  ;; Move to -1.0 rad
  (ros::sleep 2)

  ;; Final status
  (ros::ros-info "Final status:")
  (send *scissor* :print-status)

  (ros::ros-info "Basic example completed!"))

;; Run the example if this file is executed directly
(when (string= (car (last lisp::*eustop-argument*)) "scissor-example-basic.l")
  (scissor-basic-example))