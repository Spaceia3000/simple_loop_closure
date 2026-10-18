# SimpleLoopClosure V1 — Checkpoint de continuidad (18-10-2026)

## Identidad y conservación

- Candidato FUNCIONAL UNIFICADO  (base , C1 merge , operador/RViz ). Contiene S1 , S2  configurable con default 10, y S3  configurable con default 1. Los valores 100/10 se proporcionan mediante YAML experimental, NO son defaults de producción.
- Base estable SLC , intacta; FAST-LIVO2 , intacto.
- C1 certificación fuente+testigo+grabador+PGO+grafo: código en rama , integrado en candidata . Gate  requiere timestamps ordenados e idénticos, cero faltantes/extras/duplicados y auditorías nativas. No fija 2660: deriva cardinalidad del rosbag, verifica metadatos de tópicos, permite  opcional. 93 tests en rama C1, 102 Python y 103 tests ROS en unificada, todos PASS.
- Guía/launch/RViz validada , derivada de  e integrada en candidata. El TF de FAST NO es TF corregido PGO. LO no certificado.

## Evidencia real CBD03 LIO, 1.0x nominal

- Tres ejecuciones backend @3f35f20 con fuente 2660 parejas, SLC recibe y sincroniza 2660, publica y graba PGO 2660. Todas: 351 keyframes, 176 búsquedas, 26 factores de cierre, 377 factores GTSAM; digest búsquedas , cierres ; auditorías nativas PASS. Tasa efectiva ~0.9969x, 0.9983x, 0.9984x.
- NO 3/3 C1 global: tercera ejecución  testigo cloud 2637/2660, grabador cloud 2612/2660 aunque backend y PGO completos. C1  lo debe clasificar FAIL. Las dos primeras ejecuciones son PASS externo.
- CBD03 LIVO previo @reader100, fuente 2377; backend 2341, testigo cloud 2309, recorder cloud 2305, PGO 2341. Las pérdidas coinciden con congestión temporal ~178–185s, pero diferentes receptores perdieron diferentes timestamps. FAIL global. Documentado en . DDS instrumentación opcional lista  (111 tests) AÚN sin replay de validación causal.

## Prueba estricta del candidato unificado @00f819e

Origen: rosbag fijo  con 301 pares.

- Con  y , corrida : FAIL correctamente, backend solo 280 cloud/sync/PGO, aunque fuente, testigo, recorder recibieron 301 cloud y 301 odom. Los 21 ausentes son índices terminales 280..300 y el reproductor marcó velocidad efectiva ~0.95268x. No se registraron .
- Mismo binario, misma configuración y fuente, solo , corrida : PASS. Fuente/testigo/recorder 301 nube+odom, PGO testigo/recorder 301, backend nativo y auditoría externa PASS, 4/4 procesos retornan 0. Un proceso ajeno de calibración LiDAR–cámara estaba activo: esto limita causalidad perfecta entre ejecuciones.
- **No se establece que 25 s sea universal ni certificación de full dataset bajo **. Próxima mejora operativa recomendada: gate de drenaje dependiente de recepción completa + watchdog acotado antes de enviar SIGINT; no usar espera ciega como certificación.

## F3 matemático — BLOQUEO DE CIERRE CIENTÍFICO

- Rama aislada , build PASS, 115 test records, 0 errors, 0 failures, 1 omitted expected. Arregla radio nanoflann m² vs correspondencia ICP m, excluye fuente/scans temporalmente cercanos de submapas, sustituye fitness m² como pseudo-varianza 6DoF por parámetros explícitos de varianza rotacional rad²/traslacional m² y conserva el BetweenFactor  con tests SE(3).
- En estricta ausencia de  y dos varianzas calibradas, y sin comprobación de rango/observabilidad de correspondencias ICP, la política segura deja **LOOP FACTORS DESHABILITADOS**. NO MERGEAR F3 A LA CANDIDATA FUNCIONAL ni anunciar corrección de lazo certificada hasta calibración independiente y Hessiano/rango implementados. No inventar umbrales.
- Calibrar distancias, covarianzas y criterios de rank/condition en fit/heldout con correspondencias de la ruta ICP+submapa real y comprobar residuo normalizado SE(3), consistencia, verdaderos/falsos cierres, APE/RPE con ground truth.

## Siguientes gates antes de release

1. Investigar recepción DDS bajo CBD03 LIVO y cargas adversas con telemetry opcional, y resolver integridad C1 de todos los observadores y ROS2 recorder sin falsos PASS.
2. Completar rank/observability y calibración F3, ensayos fit/heldout y factores geométricos válidos (sin backport directo de C3: su ruta ICP fue distinta).
3. Certificar seis combinaciones UGV10/Retail/CBD03 × LIO/LIVO, tres repeticiones y 1.0x real medido: 18 campañas, tiempo/memoria/SSD, exact seq/PGO/markers, grafo y métricas; LO aparte.
4. Comprobar mapa/trajectoria corregidos en RViz y UGV físico; documentación comandos sencillos en  (ROS2 Humble, localhost solo mismo equipo). Trabajar con commits/ramas aisladas, nunca tocar el main o stable hasta PASS.

**Hardware a 17:19:** SSD  ~24GB libres y 99% ocupado. No realizar 18 grabaciones de ~3GB sin planificación de almacenamiento; conservar evidencias/sha256, no borrar datos del usuario sin autorización.
