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
  (cet:enable-fluent-tracing)
  (init-belief-state)
  (with-process-modules
    (move-spine 0.2)
    (move-arms-away)
    (with-designators
        ((handle-loc (location
                      `((desig-props:pose
                         ,(tf:make-pose
                           (tf:make-3d-vector
                            -0.12 0.0 0.07)
                           (tf:euler->quaternion
                            :ax (/ pi 2)))))))
         (mug-handle (object
                      `((desig-props:type desig-props:handle)
                        (desig-props:at ,handle-loc))))
         (mug (object `((desig-props:name "mug_1")
                        (desig-props:handle ,mug-handle)))))
      (let ((mug-perceived (first (cram-plan-library:perceive-object
                                   'cram-plan-library:currently-visible
                                   mug))))
        (achieve `(cram-plan-library:object-in-hand ,mug-perceived))))))

(def-top-level-cram-function displace-mug-1 ()
  (cet:enable-fluent-tracing)
  (init-belief-state)
  (add-collision-environment)
  (with-process-modules
    (move-spine 0.2)
    (move-arms-away)
    (with-designators
        ((handle-loc (location
                      `((desig-props:pose
                         ,(tf:make-pose
                           (tf:make-3d-vector
                            -0.12 0.0 0.07)
                           (tf:euler->quaternion
                            :ax (/ pi 2)))))))
         (mug-handle (object
                      `((desig-props:type desig-props:handle)
                        (desig-props:at ,handle-loc))))
         (mug (object `((desig-props:name "mug_1")
                        (desig-props:handle ,mug-handle)))))
      (let ((mug-perceived (first (cram-plan-library:perceive-object
                                   'cram-plan-library:currently-visible
                                   mug))))
        (with-failure-handling
            ((cram-plan-failures:manipulation-pose-unreachable (f)
               (declare (ignore f))
               (retry))
             (cram-plan-failures:location-not-reached-failure (f)
               (declare (ignore f))
               (retry)))
          (add-collision-environment)
          (achieve `(cram-plan-library:object-in-hand ,mug-perceived)))
        (move-arms-away)
        (with-designators ((loc-on-table
                            (location `((desig-props:on Cupboard)
                                        (desig-props:name "popcorn_table")))))
          (with-failure-handling
              ((cram-plan-failures:manipulation-pose-unreachable (f)
                 (declare (ignore f))
                 (retry))
               (cram-plan-failures:location-not-reached-failure (f)
                 (declare (ignore f))
                 (retry)))
            (add-collision-environment)
            (achieve `(cram-plan-library:object-placed-at ,mug-perceived
                                                          ,loc-on-table))))))))
