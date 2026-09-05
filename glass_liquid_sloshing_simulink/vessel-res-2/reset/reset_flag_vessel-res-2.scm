;; reset_flag.scm — перезагрузка эталонного CASE по файлу-флагу
(define (reset-if-flag)
  (let* ((flag    "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/vessel-res-2/reset/reset_case.ok")
         (goldcas "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/vessel-res-2/reset/vessel_3.5x6.0x100_res-2.cas.h5"))

    ;; если флаг существует…
    (if (file-exists? flag)

        ;; ---------- перезагружаем CASE и убираем флаг ----------
        (begin
          ;; читаем «золотой» .cas.h5
          (ti-menu-load-string
           (string-append "/file/read-case-data " goldcas " \nOK\n"))

          ;; пробуем удалить флаг, выводим результат
          (if (remove-file flag)
              (format #t "reset.ok processed → CASE reloaded.~%")
              (format #t "Warning: can't delete reset_case.ok~%")))

        ;; ---------- иначе ничего не делаем ----------
        #f)))

