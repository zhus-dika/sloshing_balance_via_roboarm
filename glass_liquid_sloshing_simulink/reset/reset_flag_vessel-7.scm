;; reset_flag.scm ─ reload gold-CASE + standard init + patch
(define (reset-if-flag)
  (let* ((flag    "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/reset.ok")
         (goldcas "/home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/vessel_3.5x6.0x100_res-7.cas.h5"))

    (if (file-exists? flag)
        (begin
              (ti-menu-load-string
               (format #f "/file/read-case-data /home/dika/Documents/sloshing_balance_via_roboarm/glass_liquid_sloshing_simulink/reset/vessel_3.5x6.0x100_res-7.cas.h5 \nOK\n" goldcas))

              ))))
