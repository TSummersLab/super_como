
(cl:in-package :asdf)

(defsystem "zed_wrapper-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "reset_odometry" :depends-on ("_package_reset_odometry"))
    (:file "_package_reset_odometry" :depends-on ("_package"))
    (:file "reset_tracking" :depends-on ("_package_reset_tracking"))
    (:file "_package_reset_tracking" :depends-on ("_package"))
    (:file "set_initial_pose" :depends-on ("_package_set_initial_pose"))
    (:file "_package_set_initial_pose" :depends-on ("_package"))
  ))