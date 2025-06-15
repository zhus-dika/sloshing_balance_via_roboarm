(define (reset-if-flag)
  (let ((flag "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/reset.ok")          ; путь к флагу
        (init "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/initial.cas.h5"))   ; путь к снимку
    (if (file-exists? flag)
        (begin
          ;; читаем case+data
          (ti-menu-load-string
           (format #f "file/read-case-data \"~a\" yes\n" init))
          ;; убираем флаг через shell
          (system (format #f "/bin/rm -f ~a" flag))))))
