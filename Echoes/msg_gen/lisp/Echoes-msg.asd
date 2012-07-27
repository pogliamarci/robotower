
(cl:in-package :asdf)

(defsystem "Echoes-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Towers" :depends-on ("_package_Towers"))
    (:file "_package_Towers" :depends-on ("_package"))
    (:file "Rfid" :depends-on ("_package_Rfid"))
    (:file "_package_Rfid" :depends-on ("_package"))
    (:file "Sonar" :depends-on ("_package_Sonar"))
    (:file "_package_Sonar" :depends-on ("_package"))
  ))