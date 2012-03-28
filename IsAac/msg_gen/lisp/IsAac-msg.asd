
(cl:in-package :asdf)

(defsystem "IsAac-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MediaVarianza" :depends-on ("_package_MediaVarianza"))
    (:file "_package_MediaVarianza" :depends-on ("_package"))
  ))