
(cl:in-package :asdf)

(defsystem "sonar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Led" :depends-on ("_package_Led"))
    (:file "_package_Led" :depends-on ("_package"))
    (:file "Sonar" :depends-on ("_package_Sonar"))
    (:file "_package_Sonar" :depends-on ("_package"))
  ))