
(cl:in-package :asdf)

(defsystem "Echoes-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Led" :depends-on ("_package_Led"))
    (:file "_package_Led" :depends-on ("_package"))
  ))