;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Universität Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :lesson01)

(defun start-lesson ()
  (roslisp:ros-info (lesson01) "Starting lesson 01.")
  (cram-roslisp-common:startup-ros))

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        gazebo-perception-process-module:gazebo-perception-process-module
        point-head-process-module:point-head-process-module)
     ,@body))

(def-top-level-cram-function get-mug-1 ()
  (with-process-modules
    (move-spine 0.2)
    (move-arms-away)
    (with-designators
        ((mug (object `((desig-props:name "mug_1")))))
      (let ((mug-perceived (cram-plan-library:perceive-object
                            'cram-plan-library:the
                            mug)))
        (achieve `(cram-plan-library:object-in-hand ,mug-perceived))))))

(defun move-spine (position)
  (let ((spine-lift-trajectory (roslisp:make-msg
                                "trajectory_msgs/JointTrajectory"
                                (stamp header)
                                (roslisp:ros-time)
                                joint_names #("torso_lift_joint")
                                points (vector
                                        (roslisp:make-message
                                         "trajectory_msgs/JointTrajectoryPoint"
                                         positions (vector position)
                                         velocities #(0)
                                         accelerations #(0)
                                         time_from_start 5.0)))))
    (roslisp:ros-info (lesson01)
                      "Moving spine to position ~a." position)
    (pr2-manipulation-process-module::execute-torso-command
     spine-lift-trajectory)
    (roslisp:ros-info (lesson01)
                      "Moving spine complete.")))

(defun move-arms-away ()
  (roslisp:ros-info (lesson01) "Moving away arms.")
  (let* ((joint-values-left
           (list (cons "l_shoulder_pan_joint" 1.7091906456385282d0)
                 (cons "l_shoulder_lift_joint" -0.3516685176176528d0)
                 (cons "l_upper_arm_roll_joint" 0.3357926749691642d0)
                 (cons "l_elbow_flex_joint" -2.119705962915871d0)
                 (cons "l_forearm_roll_joint" 13.518303816702831d0)
                 (cons "l_wrist_flex_joint" -2.003336851649972d0)
                 (cons "l_wrist_roll_joint" -22.099177425140844d0)))
         (joint-values-right
           (list (cons "r_shoulder_pan_joint" -1.4797042291545055d0)
                 (cons "r_shoulder_lift_joint" -0.35204314859086017d0)
                 (cons "r_upper_arm_roll_joint" -0.3710246292874033d0)
                 (cons "r_elbow_flex_joint" -2.120429816472789d0)
                 (cons "r_forearm_roll_joint" 24.297695480641856d0)
                 (cons "r_wrist_flex_joint" -2.0051073464316054d0)
                 (cons "r_wrist_roll_joint" -128.81492027877437d0)))
         (msg-left
           (roslisp:make-message
            "trajectory_msgs/JointTrajectory"
            (stamp header) (roslisp:ros-time)
            joint_names (map 'vector #'car joint-values-left)
            points (vector
                    (roslisp:make-message
                     "trajectory_msgs/JointTrajectoryPoint"
                     positions (map 'vector #'cdr joint-values-left)
                     velocities (map 'vector #'identity
                                     (make-list (length joint-values-left)
                                                :initial-element 0.0))
                     accelerations (map 'vector #'identity
                                        (make-list (length joint-values-left)
                                                   :initial-element 0.0))
                     time_from_start 5.0))))
         (msg-right
           (roslisp:make-message
            "trajectory_msgs/JointTrajectory"
            (stamp header) (roslisp:ros-time)
            joint_names (map 'vector #'car joint-values-right)
            points (vector
                    (roslisp:make-message
                     "trajectory_msgs/JointTrajectoryPoint"
                     positions (map 'vector #'cdr joint-values-right)
                     velocities (map 'vector #'identity
                                     (make-list (length joint-values-right)
                                                :initial-element 0.0))
                     accelerations (map 'vector #'identity
                                        (make-list (length joint-values-right)
                                                   :initial-element 0.0))
                     time_from_start 5.0)))))
    (par
      (pr2-manip-pm::execute-arm-trajectory :left msg-left)
      (pr2-manip-pm::execute-arm-trajectory :right msg-right))
    (roslisp:ros-info (lesson01) "Moving arms complete.")))
