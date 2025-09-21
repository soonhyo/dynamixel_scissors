#!/usr/bin/env roseus
;; scissor-example-configs.l
;; Examples showing different hardware configurations

(load "scissor-interface.l")

;; Example 1: Standard hardware configuration
(defun scissor-standard-config-example ()
  "Example for standard hardware where max_position = open"

  (ros::ros-info "=== Standard Hardware Configuration Example ===")

  ;; Create scissor interface for standard hardware
  (setq *scissor-std* (instance scissor-interface :init
                               :joint-name "sample_joint"
                               :open-position 0.5      ;; max position = open
                               :close-position -3.14    ;; min position = close
                               :open-is-max t))

  ;; Wait for joint state
  (send *scissor-std* :wait-for-joint-state 3.0)

  ;; Demonstrate operations
  (ros::ros-info "Opening scissor (moving to max position)...")
  (send *scissor-std* :open 1.5)
  (ros::sleep 2)

  (ros::ros-info "Closing scissor (moving to min position)...")
  (send *scissor-std* :close 1.5)
  (ros::sleep 2)

  (send *scissor-std* :print-status)
  (ros::ros-info "Standard configuration example completed\n"))

;; Example 2: Inverted hardware configuration
(defun scissor-inverted-config-example ()
  "Example for inverted hardware where max_position = close"

  (ros::ros-info "=== Inverted Hardware Configuration Example ===")

  ;; Create scissor interface for inverted hardware
  (setq *scissor-inv* (instance scissor-interface :init
                               :joint-name "sample_joint"
                               :open-position -3.14     ;; min position = open
                               :close-position 0.5      ;; max position = close
                               :open-is-max nil))

  ;; Wait for joint state
  (send *scissor-inv* :wait-for-joint-state 3.0)

  ;; Demonstrate operations
  (ros::ros-info "Opening scissor (moving to min position)...")
  (send *scissor-inv* :open 1.5)
  (ros::sleep 2)

  (ros::ros-info "Closing scissor (moving to max position)...")
  (send *scissor-inv* :close 1.5)
  (ros::sleep 2)

  (send *scissor-inv* :print-status)
  (ros::ros-info "Inverted configuration example completed\n"))

;; Example 3: Custom positions configuration
(defun scissor-custom-config-example ()
  "Example with custom open/close positions"

  (ros::ros-info "=== Custom Positions Configuration Example ===")

  ;; Create scissor interface with custom positions
  (setq *scissor-custom* (instance scissor-interface :init
                                  :joint-name "sample_joint"
                                  :open-position 0.3       ;; custom open position
                                  :close-position -2.0     ;; custom close position
                                  :open-is-max t))

  ;; Wait for joint state
  (send *scissor-custom* :wait-for-joint-state 3.0)

  ;; Demonstrate operations
  (ros::ros-info "Opening scissor to custom position...")
  (send *scissor-custom* :open 1.5)
  (ros::sleep 2)

  (ros::ros-info "Closing scissor to custom position...")
  (send *scissor-custom* :close 1.5)
  (ros::sleep 2)

  (send *scissor-custom* :print-status)
  (ros::ros-info "Custom configuration example completed\n"))

;; Main function to run all configuration examples
(defun scissor-config-examples ()
  "Run all configuration examples"

  (ros::ros-info "Starting scissor configuration examples...")

  ;; Run each configuration example
  (scissor-standard-config-example)
  (ros::sleep 1)

  (scissor-inverted-config-example)
  (ros::sleep 1)

  (scissor-custom-config-example)

  (ros::ros-info "All configuration examples completed!"))

;; Run the examples if this file is executed directly
(when (string= (car (last lisp::*eustop-argument*)) "scissor-example-configs.l")
  (scissor-config-examples))