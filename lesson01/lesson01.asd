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

(defsystem lesson01
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :license "BSD"
  :description "A primer in pick and place. This code explains the
  basic structure of a pick and place scenario when using the CRAM
  environment."

  :depends-on (roslisp
               alexandria
               designators-ros
               cram-roslisp-common
               cram-language
               cram-reasoning
               cram-language
               cram-pr2-knowledge
               cram-plan-knowledge
               cram-environment-representation
               pr2-manipulation-knowledge
               pr2-manipulation-process-module
               pr2-reachability-costmap
               pr2-navigation-process-module
               pr2-reachability-costmap
               point-head-process-module
               object-location-designators
               physics-utils
               occupancy-grid-costmap
               location-costmap
               semantic-map-costmap
               visibility-costmap
               gazebo-perception-process-module)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "designator-config" :depends-on ("package"))
     (:file "utils" :depends-on ("package" "designator-config"))
     (:file "lesson01" :depends-on ("package" "designator-config" "utils"))))))
