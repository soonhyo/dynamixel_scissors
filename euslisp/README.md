# EusLisp Scissor Control Interface

This directory contains EusLisp interface and examples for Dynamixel scissor control.

## File Structure

- `scissor-interface.l` - Basic scissor control interface class
- `scissor-example-basic.l` - Basic usage examples
- `scissor-example-configs.l` - Various hardware configuration examples
- `scissor-example-advanced.l` - Advanced feature examples
- `scissor-robot-demo.l` - Robot integration demonstrations

## Basic Usage

### 1. Simple Open/Close Control

```lisp
;; Load interface
(load "scissor-interface.l")

;; Create scissor interface (default settings)
(setq *scissor* (instance scissor-interface :init))

;; Wait for joint state
(send *scissor* :wait-for-joint-state 5.0)

;; Open scissor (1 second duration)
(send *scissor* :open 1.0)

;; Close scissor (1 second duration)
(send *scissor* :close 1.0)

;; Move to specific position
(send *scissor* :move-to -1.5 2.0)

;; Toggle (move to farther position from current)
(send *scissor* :toggle 1.5)
```

### 2. Hardware-Specific Configuration

#### Standard Hardware (max_position = open)
```lisp
(setq *scissor-std* (instance scissor-interface :init
                             :joint-name "sample_joint"
                             :open-position 0.5      ;; max position = open
                             :close-position -3.14    ;; min position = close
                             :open-is-max t))
```

#### Inverted Hardware (max_position = close)
```lisp
(setq *scissor-inv* (instance scissor-interface :init
                             :joint-name "sample_joint"
                             :open-position -3.14     ;; min position = open
                             :close-position 0.5      ;; max position = close
                             :open-is-max nil))
```

#### Custom Position Settings
```lisp
(setq *scissor-custom* (instance scissor-interface :init
                                :joint-name "sample_joint"
                                :open-position 0.3       ;; custom open position
                                :close-position -2.0     ;; custom close position
                                :open-is-max t))
```

## Running Examples

### Basic Example
```bash
cd /path/to/dynamixel_scissors/euslisp
roseus scissor-example-basic.l
```

### Hardware Configuration Examples
```bash
roseus scissor-example-configs.l
```

### Advanced Features Example
```bash
roseus scissor-example-advanced.l
```

### Robot Integration Demo
```bash
roseus scissor-robot-demo.l
```

## Classes and Methods

### scissor-interface Class

#### Initialization Methods
- `:init` - Initialize interface
  - `:joint-name` - Joint name (default: "sample_joint")
  - `:trajectory-topic` - Trajectory goal topic
  - `:joint-states-topic` - Joint states topic
  - `:open-position` - Open position (default: 0.5)
  - `:close-position` - Close position (default: -3.14)
  - `:open-is-max` - Whether max value is open (default: t)

#### Control Methods
- `:open &optional (duration 1.0)` - Open scissor
- `:close &optional (duration 1.0)` - Close scissor
- `:move-to position &optional (duration 1.0)` - Move to specific position
- `:toggle &optional (duration 1.0)` - Toggle (open↔close)

#### Status Methods
- `:get-current-position` - Return current position
- `:get-open-position` - Return open position
- `:get-close-position` - Return close position
- `:is-open-max` - Check if max value is open
- `:get-status` - Return status information list
- `:print-status` - Print status information

#### Utility Methods
- `:wait-for-joint-state &optional (timeout 5.0)` - Wait for joint state reception
- `:joint-state-callback msg` - Joint state callback (internal use)
- `:publish-trajectory target-position &optional (duration 1.0)` - Publish trajectory

### advanced-scissor-interface Class

Extends scissor-interface with advanced features.

#### Advanced Control Methods
- `:execute-sequence movements &optional (wait-between 1.0)` - Execute movement sequence
- `:smooth-move start-pos end-pos &optional (num-steps 5) (total-duration 3.0)` - Smooth movement
- `:oscillate pos1 pos2 &optional (cycles 3) (duration-per-move 1.0)` - Oscillation motion
- `:cutting-sequence &optional (num-cuts 3) (cut-duration 0.8)` - Cutting sequence
- `:safe-move-to position &optional (duration 1.0) (min-pos -3.5) (max-pos 1.0)` - Safe movement
- `:monitor-position &optional (duration 5.0) (interval 0.5)` - Position monitoring

### robot-scissor-demo Class

Demo class for robot integration.

#### Robot Action Methods
- `:approach-object` - Object approach simulation
- `:grasp-object` - Object grasping simulation
- `:perform-cut &optional (pre-cut-delay 0.5)` - Perform cutting action
- `:release-object` - Object release simulation
- `:retract-robot` - Robot retraction simulation

#### Sequence Methods
- `:execute-cutting-sequence &optional (num-objects 3)` - Execute cutting sequence
- `:safety-check` - Safety check
- `:emergency-stop` - Emergency stop

## Real Usage Scenarios

### 1. Simple Cutting Task
```lisp
;; Prepare scissor
(setq *scissor* (instance scissor-interface :init))
(send *scissor* :wait-for-joint-state)

;; Cutting motion
(send *scissor* :open 1.0)    ;; Open
(ros::sleep 1.5)
(send *scissor* :close 0.8)   ;; Quick close
(ros::sleep 1.0)
(send *scissor* :open 1.0)    ;; Open again
```

### 2. Repetitive Cutting Task
```lisp
;; Use advanced interface
(setq *adv-scissor* (instance advanced-scissor-interface :init))
(send *adv-scissor* :cutting-sequence 5 0.6)  ;; 5 cuts, 0.6 sec interval
```

### 3. Robot-Integrated Task
```lisp
;; Run robot demo
(setq *robot-demo* (instance robot-scissor-demo :init))
(send *robot-demo* :execute-cutting-sequence 3)  ;; Process 3 objects
```

### 4. Interactive Control
```lisp
;; Run interactive demo
(interactive-scissor-demo)
```

## Safety Guidelines

1. **Position Limits**: Check actual hardware movement range and set appropriate position limits.
2. **Speed Control**: Set appropriate duration to avoid sudden movements.
3. **Emergency Stop**: Use `:emergency-stop` method when problems occur.
4. **Initial Check**: Always run `:safety-check` before operations.

## Troubleshooting

### Joint State Not Received
```lisp
;; Check topics
(ros::ros-info "Available topics:")
(ros::ros-info "~A" (ros::get-topic-list))

;; Increase timeout
(send *scissor* :wait-for-joint-state 10.0)
```

### Scissor Not Responding
```lisp
;; Check publisher
(ros::ros-info "Publishers: ~A" (send *scissor* trajectory-pub))

;; Manually check position
(ros::ros-info "Current position: ~A" (send *scissor* :get-current-position))
```

### Hardware Direction is Reversed
```lisp
;; Change open-is-max setting
(setq *scissor* (instance scissor-interface :init :open-is-max nil))
```

This interface can be used to implement scissor control for various hardware configurations and usage scenarios.