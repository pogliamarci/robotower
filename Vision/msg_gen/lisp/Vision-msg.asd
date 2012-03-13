
(cl:in-package :asdf)

(defsystem "Vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Results" :depends-on ("_package_Results"))
    (:file "_package_Results" :depends-on ("_package"))
  ))