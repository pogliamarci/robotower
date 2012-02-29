
(cl:in-package :asdf)

(defsystem "SpyKee-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Motion" :depends-on ("_package_Motion"))
    (:file "_package_Motion" :depends-on ("_package"))
    (:file "Vision" :depends-on ("_package_Vision"))
    (:file "_package_Vision" :depends-on ("_package"))
  ))