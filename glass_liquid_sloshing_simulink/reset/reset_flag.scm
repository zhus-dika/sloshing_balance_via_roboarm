;; reset_flag.scm  — перезагрузка снимка (без удаления флага)
;;-------------------------------------------------------
(define (reset-if-flag)
  (let* ((flag "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/reset.ok")
         (init "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/vessel_3.5x6.0x100_res-4.cas.h5"))
    (if (file-exists? flag)                 ; флаг существует?
        (begin
          ;; 1) читаем case+data
          (ti-menu-load-string
            (format #f "file/read-case-data \"~a\" yes\n" init))
          ;; 2) удаление флага отключено
          ;; (system (format #f "/bin/rm -f ~a" flag))
          ))))
