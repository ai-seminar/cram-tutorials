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
           (list (cons "l_shoulder_pan_joint" 1.1091906456385282d0)
                 (cons "l_shoulder_lift_joint" -0.3516685176176528d0)
                 (cons "l_upper_arm_roll_joint" 0.3357926749691642d0)
                 (cons "l_elbow_flex_joint" -1.919705962915871d0)
                 (cons "l_forearm_roll_joint" 13.518303816702831d0)
                 (cons "l_wrist_flex_joint" -1.403336851649972d0)
                 (cons "l_wrist_roll_joint" -20.099177425140844d0)))
         (joint-values-right
           (list (cons "r_shoulder_pan_joint" -1.1097042291545055d0)
                 (cons "r_shoulder_lift_joint" -0.35204314859086017d0)
                 (cons "r_upper_arm_roll_joint" -0.3710246292874033d0)
                 (cons "r_elbow_flex_joint" -1.920429816472789d0)
                 (cons "r_forearm_roll_joint" 24.297695480641856d0)
                 (cons "r_wrist_flex_joint" -1.4051073464316054d0)
                 (cons "r_wrist_roll_joint" -126.81492027877437d0)))
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

(defun get-robot-pose ()
  (let ((time (roslisp:ros-time)))
    (tf:wait-for-transform
     *tf* :time time
          :source-frame "/map"
          :target-frame "/base_link")
    (tf:transform-pose
     *tf* :pose (tf:make-pose-stamped
                 "/base_link"
                 time
                 (tf:make-identity-vector)
                 (tf:make-identity-rotation))
     :target-frame "/map")))

(defgeneric init-belief-state (&key debug-window)
  (:documentation "Initializes the CRAM bullet reasoning engine."))

(defmethod init-belief-state (&key debug-window)
  (crs:prolog `(btr:clear-bullet-world))
  (let* ((robot-pose (get-robot-pose))
         (urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
         (robot-pose robot-pose)
         (robot-rot `(,(tf:x (tf:orientation robot-pose))
                      ,(tf:y (tf:orientation robot-pose))
                      ,(tf:z (tf:orientation robot-pose))
                      ,(tf:w (tf:orientation robot-pose))))
         (robot-trans `(,(tf:x (tf:origin robot-pose))
                        ,(tf:y (tf:origin robot-pose))
                        ,(tf:z (tf:origin robot-pose))))
         (bdgs
           (car
            (force-ll
             (crs:prolog
              `(and (btr:clear-bullet-world)
                    (btr:bullet-world ?w)
                    (assert (btr:object
                             ?w btr:static-plane floor
                             ((0 0 0) (0 0 0 1))
                             :normal (0 0 1) :constant 0))
                    ,@(when debug-window
                       `((btr:debug-window ?w)))
                    (btr:robot ?robot)
                    (assert (btr:object
                             ?w btr:urdf ?robot
                             (,robot-trans ,robot-rot)
                             :urdf ,urdf-robot))))))))
         (var-value
          '?pr2
          (lazy-car
           (crs:prolog
            `(and (btr:robot ?robot)
                  (btr:%object ?w ?robot ?pr2)) bdgs)))))

(defun add-collision-environment ()
  (let* ((curr-time (roslisp:ros-time))
         (mug-2-pose (cram-gazebo-utilities:get-model-pose "mug_2"))
         (popcorn-table-pose (cram-gazebo-utilities:get-model-pose "popcorn_table"))
         (msgs (list (roslisp:make-message
                      "arm_navigation_msgs/CollisionObject"
                      (stamp header) curr-time
                      (frame_id header) (tf:frame-id mug-2-pose)
                      (id) "mug_2"
                      (padding) 0.1
                      (operation operation) 0
                      (shapes) (vector
                                (roslisp:make-message
                                 "arm_navigation_msgs/Shape"
                                 (type) 2
                                 (dimensions) (vector 0.1 0.2)))
                      (poses) (vector (tf:pose->msg
                                       (tf:make-pose
                                        (tf:v+ (tf:origin mug-2-pose)
                                               (tf:make-3d-vector 0 0 0.1))
                                        (tf:orientation mug-2-pose)))))
                     (roslisp:make-message
                      "arm_navigation_msgs/CollisionObject"
                      (stamp header) curr-time
                      (frame_id header) (tf:frame-id mug-2-pose)
                      (id) "popcorn_table"
                      (padding) 0.0
                      (operation operation) 0
                      (shapes) (vector
                                (roslisp:make-message
                                 "arm_navigation_msgs/Shape"
                                 (type) 1
                                 (dimensions) (vector 0.64 1.75 0.9)))
                      (poses) (vector (tf:pose->msg
                                       (tf:make-pose
                                        (tf:v+ (tf:origin popcorn-table-pose)
                                               (tf:make-3d-vector 0 0 0.45))
                                        (tf:orientation popcorn-table-pose))))))))
    (loop for msg in msgs
          do (roslisp:publish
              (roslisp:advertise
               "/collision_object"
               "arm_navigation_msgs/CollisionObject")
              msg))))
