#!/usr/bin/env roseus
;; scissor-example-advanced.l
;; Advanced usage examples with trajectory control and sequences

(load "scissor-interface.l")

;; Advanced scissor interface with additional features
(defclass advanced-scissor-interface
  :super scissor-interface
  :slots (trajectory-buffer))

(defmethod advanced-scissor-interface
  (:init (&rest args)
    (send-super* :init args)
    (setq trajectory-buffer nil)
    self)

  ;; Execute a sequence of movements
  (:execute-sequence (movements &optional (wait-between 1.0))
    "Execute a sequence of movements with specified delays"
    (dolist (movement movements)
      (let ((position (car movement))
            (duration (if (cdr movement) (cadr movement) 1.0)))
        (ros::ros-info "Moving to position ~A with duration ~A" position duration)
        (send self :move-to position duration)
        (ros::sleep (+ duration wait-between))))
    (ros::ros-info "Sequence completed"))

  ;; Smooth interpolation between positions
  (:smooth-move (start-pos end-pos &optional (num-steps 5) (total-duration 3.0))
    "Move smoothly between two positions with intermediate steps"
    (let ((step-size (/ (- end-pos start-pos) num-steps))
          (step-duration (/ total-duration num-steps)))
      (dotimes (i (1+ num-steps))
        (let ((current-pos (+ start-pos (* i step-size))))
          (ros::ros-info "Smooth step ~A/~A: moving to ~A" (1+ i) (1+ num-steps) current-pos)
          (send self :move-to current-pos step-duration)
          (ros::sleep step-duration)))))

  ;; Oscillate between two positions
  (:oscillate (pos1 pos2 &optional (cycles 3) (duration-per-move 1.0))
    "Oscillate between two positions for specified cycles"
    (dotimes (i cycles)
      (ros::ros-info "Oscillation cycle ~A/~A" (1+ i) cycles)
      (send self :move-to pos1 duration-per-move)
      (ros::sleep (+ duration-per-move 0.2))
      (send self :move-to pos2 duration-per-move)
      (ros::sleep (+ duration-per-move 0.2)))
    (ros::ros-info "Oscillation completed"))

  ;; Perform cutting motion sequence
  (:cutting-sequence (&optional (num-cuts 3) (cut-duration 0.8))
    "Perform a cutting sequence with repeated open/close motions"
    (let ((open-pos (send self :get-open-position))
          (close-pos (send self :get-close-position)))
      (ros::ros-info "Starting cutting sequence with ~A cuts" num-cuts)

      ;; Start from open position
      (send self :open 1.0)
      (ros::sleep 1.2)

      ;; Perform cutting motions
      (dotimes (i num-cuts)
        (ros::ros-info "Cut ~A/~A" (1+ i) num-cuts)
        (send self :close cut-duration)
        (ros::sleep (+ cut-duration 0.1))
        (send self :open cut-duration)
        (ros::sleep (+ cut-duration 0.1)))

      (ros::ros-info "Cutting sequence completed")))

  ;; Safe position check before movement
  (:safe-move-to (position &optional (duration 1.0) (min-pos -3.5) (max-pos 1.0))
    "Move to position with safety checks"
    (if (and (>= position min-pos) (<= position max-pos))
        (progn
          (ros::ros-info "Safe move to position ~A" position)
          (send self :move-to position duration))
      (progn
        (ros::ros-warn "Position ~A is outside safe range [~A, ~A]" position min-pos max-pos)
        nil)))

  ;; Position monitoring
  (:monitor-position (&optional (duration 5.0) (interval 0.5))
    "Monitor position changes for specified duration"
    (let ((start-time (ros::time-now))
          (positions nil))
      (while (< (send (ros::time- (ros::time-now) start-time) :to-sec) duration)
        (ros::spin-once)
        (push (list (send (ros::time- (ros::time-now) start-time) :to-sec)
                    (send self :get-current-position)) positions)
        (ros::ros-info "Position at t=~A: ~A rad"
                      (send (ros::time- (ros::time-now) start-time) :to-sec)
                      (send self :get-current-position))
        (ros::sleep interval))
      (reverse positions)))
)

;; Example 1: Sequence execution
(defun advanced-example-1 ()
  "Example showing sequence execution"

  (ros::ros-info "=== Advanced Example 1: Sequence Execution ===")

  (setq *adv-scissor* (instance advanced-scissor-interface :init))
  (send *adv-scissor* :wait-for-joint-state 3.0)

  ;; Define a sequence: (position duration)
  (setq movement-sequence '((-1.0 1.5)    ;; Move to -1.0 rad in 1.5 sec
                           (0.0 1.0)      ;; Move to 0.0 rad in 1.0 sec
                           (-2.0 2.0)     ;; Move to -2.0 rad in 2.0 sec
                           (0.2 1.0)))    ;; Move to 0.2 rad in 1.0 sec

  (send *adv-scissor* :execute-sequence movement-sequence 0.5)
  (ros::ros-info "Sequence example completed\n"))

;; Example 2: Smooth movement
(defun advanced-example-2 ()
  "Example showing smooth interpolated movement"

  (ros::ros-info "=== Advanced Example 2: Smooth Movement ===")

  (unless (boundp '*adv-scissor*)
    (setq *adv-scissor* (instance advanced-scissor-interface :init))
    (send *adv-scissor* :wait-for-joint-state 3.0))

  ;; Smooth movement from current position to target
  (let ((start-pos (send *adv-scissor* :get-current-position))
        (end-pos -2.5))
    (ros::ros-info "Smooth movement from ~A to ~A" start-pos end-pos)
    (send *adv-scissor* :smooth-move start-pos end-pos 8 4.0))

  (ros::ros-info "Smooth movement example completed\n"))

;; Example 3: Oscillation
(defun advanced-example-3 ()
  "Example showing oscillation between positions"

  (ros::ros-info "=== Advanced Example 3: Oscillation ===")

  (unless (boundp '*adv-scissor*)
    (setq *adv-scissor* (instance advanced-scissor-interface :init))
    (send *adv-scissor* :wait-for-joint-state 3.0))

  ;; Oscillate between two positions
  (send *adv-scissor* :oscillate -1.0 0.3 4 0.8)
  (ros::ros-info "Oscillation example completed\n"))

;; Example 4: Cutting sequence
(defun advanced-example-4 ()
  "Example showing cutting motion sequence"

  (ros::ros-info "=== Advanced Example 4: Cutting Sequence ===")

  (unless (boundp '*adv-scissor*)
    (setq *adv-scissor* (instance advanced-scissor-interface :init))
    (send *adv-scissor* :wait-for-joint-state 3.0))

  ;; Perform cutting sequence
  (send *adv-scissor* :cutting-sequence 5 0.6)
  (ros::ros-info "Cutting sequence example completed\n"))

;; Example 5: Position monitoring
(defun advanced-example-5 ()
  "Example showing position monitoring during movement"

  (ros::ros-info "=== Advanced Example 5: Position Monitoring ===")

  (unless (boundp '*adv-scissor*)
    (setq *adv-scissor* (instance advanced-scissor-interface :init))
    (send *adv-scissor* :wait-for-joint-state 3.0))

  ;; Start a movement and monitor position
  (send *adv-scissor* :move-to -2.0 3.0)
  (let ((positions (send *adv-scissor* :monitor-position 4.0 0.3)))
    (ros::ros-info "Recorded ~A position samples" (length positions)))

  (ros::ros-info "Position monitoring example completed\n"))

;; Main function for all advanced examples
(defun scissor-advanced-examples ()
  "Run all advanced examples"

  (ros::ros-info "Starting advanced scissor examples...")

  ;; Run all examples
  (advanced-example-1)
  (ros::sleep 1)

  (advanced-example-2)
  (ros::sleep 1)

  (advanced-example-3)
  (ros::sleep 1)

  (advanced-example-4)
  (ros::sleep 1)

  (advanced-example-5)

  (ros::ros-info "All advanced examples completed!"))

;; Run the examples if this file is executed directly
(when (string= (car (last lisp::*eustop-argument*)) "scissor-example-advanced.l")
  (scissor-advanced-examples))