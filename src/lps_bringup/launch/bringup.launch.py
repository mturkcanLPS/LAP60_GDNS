from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction, ExecuteProcess,
                            DeclareLaunchArgument, SetEnvironmentVariable, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration,
                                  PythonExpression)
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue


# ---------------------------------------------------------------------------
# infra1 (ucus) ve infra2 (test) AYNI hesabi yapmali ki videolar kiyaslanabilsin.
# Asagidaki sozlukler iki tarafa da ORTAK verilir — birini degistirmek ikisini
# birden degistirir, kayma olamaz.
# ---------------------------------------------------------------------------
APRILTAG_DETECT = {                  # cpu_apriltag (infra1) + cpu_apriltag_infra2
    'size':              0.245,      # tag25h9 huni duvar buyuk tag = 250mm
    'quad_decimate':     1.0,        # tam cozunurluk (kucuk/uzak tag)
    'quad_sigma':        0.0,
    'decode_sharpening': 1.0,        # bulanik/egik tag icin
    'refine_edges':      True,
}
# serial_sender + infra2_recorder: tag -> avgPos fuzyonunda kullanilan node size.
# DIKKAT: tag_fusion.hpp 'NOT: 45/24 modunda node size 0.245 olmali' diyor;
# APRILTAG_DETECT['size'] 0.245 iken burasi 0.25 (degisiklik oncesiyle ayni
# birakildi, bilincli karar bekliyor).
FUSION_NODE_SIZE = 0.25
FLOW = {                             # serial_sender + infra2_recorder
    'grid_size':            4,
    # Piramit seviyesi = izlenebilir MAKS kayma (her seviye 2 kat):
    # lv3 -> 32 px, lv4 -> 64 px. Kirilma altinda dogruluga etkisi yok,
    # bedavaya tavan yukselir; hizli harekette veri kaybini onler (LPS).
    'num_levels':           4,
    'ema_alpha':            0.25,
    'flow_texture_thresh':  4.0,     # node varsayilani (acik yazildi: ortak olsun)
    'flow_prefilter_sigma': 1.5,     # node varsayilani (acik yazildi: ortak olsun)
}
# infra2 veri yolu: ureten node'lar ile serial_sender AYNI isimleri kullanmali
INFRA2_TAG_TOPIC  = '/tag_detections_infra2'   # cpu_apriltag_infra2 -> serial aprtiltag2
INFRA2_FLOW_TOPIC = '/infra2/optical_flow'     # infra2_recorder     -> serial opticflow2
# Arac tespiti (car_detector: DeepStream YOLOv8 VisDrone + NvDCF) -> paketin sonundaki car.
# Model dizini DeepStream-Yolo altinda: config + ONNX + labels + TensorRT motoru BIRLIKTE.
# (Ozel YOLO kutuphanesi motoru CALISMA dizinine 'model_b1_gpu0_fp16.engine' yazar;
#  car_detector config'in dizinine chdir ettigi icin motor burada olusur ve sonraki
#  acilislarda yeniden kullanilir. Kurulum: car_detector/scripts/setup_visdrone_model.sh)
# Model secimi (22_10): launch arg car_model. Ayni agirliklar (test_visdrone_best.pt),
# farkli giris boyutu; her biri kendi klasorunde ONNX + config + ONCEDEN DERLENMIS motor.
# VisDrone val olcumu (1280x720'ye olcekli): toplam yakalama 640 %66.6 -> 736x1280 %79.0,
# menzil ~2x (60 m altinda fark yok). Maliyet: GPU +5.4 ms/kare, +0.7 W; CPU/RAM ayni.
CAR_MODEL_ROOT = '/home/lapis/DeepStream-Yolo'
CAR_MODELS = {'640': 'visdrone', '960': 'visdrone_960',
              '736x1280': 'visdrone_736x1280', '1280': 'visdrone_1280'}
CAR_MODEL_DEFAULT = '736x1280'
CAR_MODEL_FALLBACK = '640'           # bilinmeyen isimde yigini DUSURME, eski modele don
CAR_CONFIG_NAME = 'config_infer_primary_yoloV8_visdrone.txt'
CAR_ENGINE_NAME = 'model_b1_gpu0_fp16.engine'


def car_model_paths(name):
    """car_model adi -> (gercek ad, nvinfer config yolu, motor var mi). Bilinmeyen ad
    -> 640'a duser (launch hata verip tum servisi oldurmesin)."""
    import os
    real = name if name in CAR_MODELS else CAR_MODEL_FALLBACK
    d = os.path.join(CAR_MODEL_ROOT, CAR_MODELS[real])
    return real, os.path.join(d, CAR_CONFIG_NAME), os.path.isfile(os.path.join(d, CAR_ENGINE_NAME))
CAR_BEARING_TOPIC  = '/car_detection/bearing'  # car_detector -> serial car
RECORD = {                           # iki video da ayni klasor/fps/olcek
    'record_dir':   '/home/lapis/lps_records',
    'record_fps':   15.0,
    'record_scale': 0.5,
}
SESSION_FILE = '.oturum_sayaci'      # record_dir icinde, son oturum numarasi


def next_record_session(record_dir):
    """Kayit oturum numarasi: servis (launch) her acilista +1. Tum kayit dosyalari
    <onek>_sNNNN_... adini alir (record_naming.hpp). Neden: Jetson acilista saati geri
    yukluyor (sahada NTP yok) -> tarihli adlar tekrar ediyor ve 20_10'da yeni acilis
    19_10 ucus kayitlarinin USTUNE yazdi.
    - Sayac dosyasi fsync'li: ani guc kesiminde bile numara geri gitmez.
    - Klasordeki en buyuk _sNNNN de hesaba katilir (sayac silinse/geride kalsa bile).
    - Yazilamazsa (or. masa testi lapis, sayac root'un) yazmadan max+1 kullanilir;
      node'lar adi O_EXCL ile sahiplendigi icin cakisma yine de ustune yazmaz."""
    import fcntl, os, re
    os.makedirs(record_dir, exist_ok=True)
    seen = 0
    for f in os.listdir(record_dir):
        m = re.search(r'_s(\d{4,})_', f)
        if m:
            seen = max(seen, int(m.group(1)))
    path = os.path.join(record_dir, SESSION_FILE)
    try:
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            txt = os.read(fd, 64).decode(errors='ignore').strip()
            n = max(int(txt) if txt.isdigit() else 0, seen) + 1
            os.lseek(fd, 0, os.SEEK_SET)
            os.ftruncate(fd, 0)
            os.write(fd, f'{n}\n'.encode())
            os.fsync(fd)
        finally:
            os.close(fd)
        dfd = os.open(record_dir, os.O_RDONLY)   # yeni olusan sayac dosyasinin dizin kaydi
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
        return n
    except OSError:
        try:                                     # yazamasak da okuyabiliyorsak sayaci gec
            with open(path) as fh:
                txt = fh.read().strip()
            seen = max(seen, int(txt) if txt.isdigit() else 0)
        except OSError:
            pass
        return seen + 1


def generate_launch_description():
    record_session = next_record_session(RECORD['record_dir'])
    # AprilTag arka ucu: apriltag_backend:=cpu (VARSAYILAN, libapriltag) veya
    #   :=gpu (Isaac/cuAprilTags). Ikisi de /tag_detections'a yayinlar; serial_sender
    #   (hrvs + funnel + flow) hangisi acilirsa onu tuketir, degismez.
    backend_arg = DeclareLaunchArgument('apriltag_backend', default_value='cpu')
    # Huni tag25h9 -> default. Baska aile denemek icin tag_family:=... ile ez.
    tag_family_arg = DeclareLaunchArgument('tag_family', default_value='tag36h11')
    # IKINCI IR KAMERA (infra2): acilir, uzerinde AprilTag + optik akis
    # hesaplanir; sonuclar paketin SONUNDAKI aprtiltag2 / opticflow2 alanlariyla
    # SERIALE GIDER (infra1 alanlariyla birlestirilmez, ayri alanlar) ve ayrica
    # lps_infra2_*.avi'ye overlay'li kaydedilir. 'false' -> infra2 kapanir,
    # aprtiltag2/opticflow2 SIFIR gider (paket boyutu degismez, 120 B).
    # Servis argumansiz calistigi icin varsayilan burada. Ek CPU + OFA yuku getirir.
    infra2_test_arg = DeclareLaunchArgument('infra2_test', default_value='true')
    # Arac tespiti (renkli kamera + DeepStream YOLO). Bearing paketin SONUNDAKI car
    # alaniyla gider. 'false' -> renkli kamera da kapanir, car alani SIFIR (valid=0).
    enable_car_arg = DeclareLaunchArgument('enable_car', default_value='true')
    # Arac videosu (eo_*.avi): ac/kapa ve olcek. OLCULDU 28_09 (tam yigin, 60 s):
    #   kayit      car_detector CPU   toplam   akis atlanan kare (infra1/infra2)
    #   kapali          %15            %76         %0.8 / %0.5
    #   1.00 (eski)     %78            %86         %5.0 / %3.3
    #   0.75            %73            %86         %3.5 / %2.2   <- INTER_AREA kesirli
    #   0.50            %33            %79         %1.3 / %0.7      oranda yavas yol
    # 0.5 = en verimli nokta (tam-sayi oranli kucultme hizli) + infra videolariyla
    # ayni boyut (640x360). Tespit tam cozunurlukte, olcek sadece videoyu etkiler.
    car_record_arg = DeclareLaunchArgument('car_record', default_value='true')
    car_record_scale_arg = DeclareLaunchArgument('car_record_scale', default_value='0.5')
    # nvvideoconvert yuzey kopyasi: 0 = varsayilan (GPU), 2 = VIC. 19_10 ucusundaki GPU
    # MMU hatasi (kopya motoru ce0, NULL adres) icin DENEME — 20_10 kullanici istegiyle acik.
    car_copy_hw_arg = DeclareLaunchArgument('car_copy_hw', default_value='2')
    car_model_arg = DeclareLaunchArgument('car_model', default_value=CAR_MODEL_DEFAULT,
        description='Arac modeli giris boyutu: ' + ' | '.join(CAR_MODELS))
    # Kamera kare hizi (IR 1280x720: 6 / 15 / 30). Iki kamera donanimda senkron;
    # akis kare-araligi istatistigi de bunu kullanir.
    camera_fps_arg = DeclareLaunchArgument('camera_fps', default_value='30')
    # true (VARSAYILAN, 28_09): infra2 AprilTag infra1 ile ESIT (nice yok, nthreads 4)
    # -> iki kamera ayni hizda. false: nice 10 + nthreads 2 -> infra2 tag infra1'in
    # ~yarisi hizda kalir (olculdu: 29.9 / 15.8 Hz).
    infra2_equal_arg = DeclareLaunchArgument('infra2_equal_priority', default_value='true')
    # Seri cikis. Olcum/masa testinde sahte porta (pty) yonlendirmek icin —
    # otopilota henuz tanimadigi paket gitmesin.
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS1')
    CAMERA_FPS = ParameterValue(LaunchConfiguration('camera_fps'), value_type=float)
    # AprilTag'i her N. karede calistir (IKI kamerada da, AYNI karelerde). 1 = her
    # kare. VARSAYILAN 2 (28_09, olcum + kullanici onayi): tag iki kamerada sabit
    # 15 Hz, akis iki kamerada sabit 30 Hz (%99 duzenli), CPU %77. Karsilastirma
    # (30 fps, donanim saatleri tavanda):
    #   every_n=1 + esit oncelik : tag 21.7 / 20.8 Hz (sabit degil), CPU %92 doygun
    #   every_n=2 + esit oncelik : tag 15.0 / 15.0 Hz, akis 30 / 30 Hz
    # NOT: donanim saatleri (scripts/set_vision_clocks.sh, servis ExecStartPre)
    # olmadan akis 30 Hz'e YETISMEZ.
    apriltag_every_n_arg = DeclareLaunchArgument('apriltag_every_n', default_value='2')
    APRILTAG_EVERY_N = ParameterValue(LaunchConfiguration('apriltag_every_n'), value_type=int)
    # Kamera karelerini paylasimli bellekten tasiyan FastDDS profili (config/
    # fastdds_shm.xml). OLCULDU 28_09: profilsiz 921 KB'lik kareler UDP'ye dusup
    # tampon tasmasindan kayboluyordu (%17.8 UDP datagrami RcvbufErrors; 30 Hz
    # kameradan 18-24 Hz alinabiliyordu). Profille UDP kaybi 0, iki node da 30.0 Hz
    # aliyor. Geri almak icin: fastdds_shm:=false
    fastdds_shm_arg = DeclareLaunchArgument('fastdds_shm', default_value='true')
    set_fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        PathJoinSubstitution([FindPackageShare('lps_bringup'), 'config', 'fastdds_shm.xml']),
        condition=IfCondition(LaunchConfiguration('fastdds_shm')))

    # 1) RealSense - IR açık, emitter kapalı
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            # Renkli kamera SADECE arac tespiti icin (enable_car). YOLO modeli RGB
            # bekliyor; diger dronedaki gibi 1280x720 @ 15 fps.
            'enable_color': LaunchConfiguration('enable_car'),
            'rgb_camera.color_profile': '1280x720x15',
            'enable_depth': 'false',
            'enable_infra1': 'true',
            # infra2 sadece test kaydi icin (infra2_test). Emitter kapatma ve
            # infra_profile ikisine birden uygulanir; pozlama da ORTAK (D4xx).
            'enable_infra2': LaunchConfiguration('infra2_test'),
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'depth_module.infra_profile': ['1280x720x', LaunchConfiguration('camera_fps')],
            'depth_module.depth_profile': ['1280x720x', LaunchConfiguration('camera_fps')],
        }.items()
    )

    # IR emitter'i kapat — rs_launch declared arg listesinde olmadigi icin
    # launch arg ile gecmiyor, runtime'da set ediyoruz.
    disable_emitter = ExecuteProcess(
        cmd=['ros2', 'param', 'set',
             '/camera/camera',
             'depth_module.emitter_enabled', '0'],
        output='screen',
    )

    # 2a) GPU AprilTag (Isaac): mono→rgb + AprilTag. SADECE apriltag_backend:=gpu.
    #     NOT: Isaac/cuAprilTags tag25h9'u desteklemeyebilir; GPU'ya gecilecekse
    #     family destegi dogrulanmali (aktif yol CPU).
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('apriltag_backend'), "' == 'gpu'"])),
        composable_node_descriptions=[
            # mono8 → rgb8 dönüştürücü
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='mono_to_rgb',
                parameters=[{
                    'encoding_desired': 'rgb8',
                }],
                remappings=[
                    ('image_raw', '/camera/camera/infra1/image_rect_raw'),
                    ('image', '/camera/camera/infra1/image_rgb'),
                ],
            ),
            # AprilTag - dönüştürülmüş rgb8 topic'ten alır
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                remappings=[
                    ('image', '/camera/camera/infra1/image_rgb'),
                    ('camera_info', '/camera/camera/infra1/camera_info'),
                ],
                parameters=[{
                    # Huni duvar buyuk tag siyah kare = 250mm (funnel scale=1.0).
                    'size': 0.25,
                    'tag_family': 'tag25h9',
                    'max_tags': 64,
                    'tile_size': 4,
                }],
            ),
        ],
    )

    # 2b) CPU AprilTag (libapriltag): SADECE apriltag_backend:=cpu (VARSAYILAN).
    #     tam cozunurluk (quad_decimate=1) + decode_sharpening ile kucuk/uzak/oblik
    #     tag'i Isaac'ten iyi yakalar. nthreads=4 (6 cekirdekten): yogun sahnede drop
    #     azaltir. /tag_detections'a yayinlar (serial_sender funnel bunu tuketir).
    cpu_apriltag = Node(
        package='lps_serial_sender', executable='cpu_apriltag_node',
        name='cpu_apriltag', output='screen',
        respawn=True, respawn_delay=2.0,   # cokerse ~2 s'de kendi kalkar (20_10 olayi)
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('apriltag_backend'), "' == 'cpu'"])),
        parameters=[{
            **APRILTAG_DETECT,
            'image_topic':       '/camera/camera/infra1/image_rect_raw',
            'camera_info_topic': '/camera/camera/infra1/camera_info',
            'output_topic':      '/tag_detections',
            'tag_family':        LaunchConfiguration('tag_family'),
            'process_every_n':   APRILTAG_EVERY_N,
            'camera_fps':        CAMERA_FPS,
            'nthreads':          4,
        }],
    )

    # 2c) infra2: ikinci CPU AprilTag ornegi — AYNI tespit ayarlari
    #     (APRILTAG_DETECT), ayri topic -> serial_sender aprtiltag2 + infra2 videosu.
    #     nthreads sonucu degil sadece hizi etkiler; infra1'den cekirdek calmasin
    #     diye 2 + dusuk oncelik (nice 10). BEDELI: infra2 tag hizi ~7-8 Hz
    #     (infra1 ~21-26 Hz) -> aprtiltag2 daha seyrek guncellenir.
    equal = LaunchConfiguration('infra2_equal_priority')
    cpu_apriltag_infra2 = Node(
        package='lps_serial_sender', executable='cpu_apriltag_node',
        name='cpu_apriltag_infra2', output='screen',
        respawn=True, respawn_delay=2.0,   # cokerse ~2 s'de kendi kalkar (20_10 olayi)
        prefix=PythonExpression(["'' if '", equal, "' == 'true' else 'nice -n 10'"]),
        condition=IfCondition(LaunchConfiguration('infra2_test')),
        parameters=[{
            **APRILTAG_DETECT,
            'image_topic':       '/camera/camera/infra2/image_rect_raw',
            'camera_info_topic': '/camera/camera/infra2/camera_info',
            'output_topic':      INFRA2_TAG_TOPIC,
            'tag_family':        LaunchConfiguration('tag_family'),
            'process_every_n':   APRILTAG_EVERY_N,
            'camera_fps':        CAMERA_FPS,
            'nthreads':          ParameterValue(PythonExpression(
                                     ["4 if '", equal, "' == 'true' else 2"]), value_type=int),
        }],
    )

    # 2d) infra2 akis + kayit: VPI akis -> INFRA2_FLOW_TOPIC (serial_sender
    #     opticflow2'ye koyar) + tag fuzyonu/akis overlay -> lps_infra2_*.avi.
    #     Seri portu kendisi yazmaz; paketin tek yazari serial_sender.
    #     nice YOK — bilerek: 'nice -n 10' ile bu node 30 Hz'lik goruntunun
    #     sadece 4-9 Hz'ini alabiliyordu (dusuk oncelik DDS alim thread'lerini de
    #     yavaslatiyor, 921 KB'lik mesajlar dusuyor). Ayni anda olculdu: nice 0
    #     -> 25-27 Hz. Node hafif (goruntu kopyasi + kayit + VPI); nice 0 iken
    #     ucus node'u OPTICFLOW 23.3->22.5, APRILTAG 24.6->22.8 Hz (saniyelik
    #     oynamanin 17-29 Hz icinde). Agir olan AprilTag kopyasi nice 10'da kaliyor.
    infra2_recorder = Node(
        package='lps_serial_sender', executable='infra2_recorder_node',
        name='infra2_recorder', output='screen',
        respawn=True, respawn_delay=2.0,   # cokerse ~2 s'de kendi kalkar (20_10 olayi)
        condition=IfCondition(LaunchConfiguration('infra2_test')),
        parameters=[{
            **FLOW,
            **RECORD,
            'image_topic':        '/camera/camera/infra2/image_rect_raw',
            'tag_topic':          INFRA2_TAG_TOPIC,
            'flow_topic':         INFRA2_FLOW_TOPIC,
            'camera_fps':         CAMERA_FPS,
            'apriltag_node_size': FUSION_NODE_SIZE,
            'record_prefix':      'lps_infra2',
            'record_session':     record_session,
            'record_label':       'INFRA2 (seriale giden: aprtiltag2/opticflow2)',
            # OFA donanimi infra1 akisiyla paylasiliyor. serial_sender logundaki
            # '[OPTICFLOW] Hz' ~30'un altina duserse False yap (sadece tag kaydi).
            'flow_enabled':       True,
        }],
    )

    # 2e) Arac tespiti: renkli kamera -> DeepStream (YOLOv8 VisDrone + NvDCF) -> tek
    #     arac kilidi -> bearing (X/Z, Y/Z) -> CAR_BEARING_TOPIC -> serial 'car'.
    #     Intrinsics camera_info'dan (fx=0): diger dronedaki SABIT degerler o dronun
    #     kamerasina aitti; kamera degisse de camera_info dogru kalir.
    #     DAYANIKLILIK (19_10): GPU hatasi / takilma / RSS > max_rss_mb'de node kendini
    #     kapatir, respawn ~2 s sonra yeniden acar (motor onbellekte, ~1 s). Diger
    #     node'lar etkilenmez; serial 'car' alani car_timeout_ms ile valid=0 olur.
    def make_car_detector(context):
        # Model yolu launch aninda cozulur; motor yoksa ilk acilista derlenecek (~6 dk,
        # car_detector ~1850 MB) -> RAM bekcisi o acilis icin kapali, yoksa derlemeyi keser.
        if LaunchConfiguration('enable_car').perform(context).lower() not in ('true', '1'):
            return []
        want = LaunchConfiguration('car_model').perform(context)
        name, cfg, have_engine = car_model_paths(want)
        acts = [LogInfo(msg=f'car_detector modeli: {name} -> {cfg}')]
        if name != want:
            acts.append(LogInfo(msg=f'UYARI: bilinmeyen car_model "{want}" -> {name} kullaniliyor'))
        if not have_engine:
            acts.append(LogInfo(msg='UYARI: motor yok, ilk acilista ~6 dk derlenecek; RAM bekcisi kapali'))
        acts.append(
        Node(
            package='car_detector', executable='car_detector_node',
            name='car_detector', output='screen',
            respawn=True, respawn_delay=2.0,
            condition=IfCondition(LaunchConfiguration('enable_car')),
            parameters=[{
                'nvinfer_config':  cfg,
                'image_topic':       '/camera/camera/color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                # VisDrone: car + van + truck. van 02_11 eklendi: hedef Hilux (kasasi yesil
                # cisimle kapli) pikap oldugu icin van sayilabilir — kilit kacmasin.
                'vehicle_classes': [3, 4, 5],
                'mux_width':       1280,
                'mux_height':      720,
                'record_video':    ParameterValue(LaunchConfiguration('car_record'), value_type=bool),
                'record_scale':    ParameterValue(LaunchConfiguration('car_record_scale'), value_type=float),
                'record_dir':      RECORD['record_dir'],
                'record_session':  record_session,
                'record_fps':      15.0,         # renkli kamera 15 fps -> gercek hizda oynar
                'copy_hw':         ParameterValue(LaunchConfiguration('car_copy_hw'), value_type=int),
                'max_rss_mb':      1500 if have_engine else 0,   # normal ~560 MB; motor derlenirken ~1850
                'stall_timeout_s': 3.0,
            }],
        )
        )
        return acts

    car_detector = OpaqueFunction(function=make_car_detector)

    # 3) Birleşik serial sender: hrvs + AprilTag FUNNEL + VPI OpticalFlow → serial
    serial_sender = Node(
        package='lps_serial_sender',
        executable='serial_sender_node',
        name='serial_sender',
        # Otopilota paket gonderen TEK node: cokerse 2 s'de yeniden acilir (radar reset
        # darbesiyle temiz baslar). 20_10: saat 1970->2026 atlarken negatif kamera damgasi
        # serial_sender + apriltag'leri + infra2_recorder'i oldurdu, otopilota 10+ dk paket
        # gitmedi (respawn yoktu).
        respawn=True, respawn_delay=2.0,   # cokerse ~2 s'de kendi kalkar (20_10 olayi)
        output='screen',
        parameters=[{
            **FLOW,      # grid/levels/ema/tex/prefilter — infra2 ile ORTAK
            **RECORD,    # record_dir/fps/scale — infra2 ile ORTAK
            'serial_port':  LaunchConfiguration('serial_port'),
            'camera_fps':   CAMERA_FPS,
            'baud_rate':    115200,
            'send_rate_hz': 50.0,
            'tag_topic':    '/tag_detections',
            'image_topic':  '/camera/camera/infra1/image_rect_raw',
            'debug':        True,
            # Huni duvar buyuk tag siyah kare = 250mm (funnel scale=1.0).
            'apriltag_node_size': FUSION_NODE_SIZE,
            # Ucus video kaydi: tespit cerceveleri + akis HUD'lu .avi (ayri
            # dusuk-oncelikli thread, uctan degil). Kapatmak icin False.
            'record_video':  True,
            'record_prefix': 'lps_infra1',   # -> lps_infra1_sNNNN_YYYYmmdd_HHMMSS.avi
            'record_session': record_session,
            'record_label':  'INFRA1 - UCUS (seriale giden)',
            # infra2 -> paketin sonundaki aprtiltag2 / opticflow2 (infra2 kapaliysa sifir)
            'infra2_tag_topic':  INFRA2_TAG_TOPIC,
            'infra2_flow_topic': INFRA2_FLOW_TOPIC,
            # arac bearing'i -> paketin sonundaki car (car_detector kapaliysa sifir)
            'car_bearing_topic': CAR_BEARING_TOPIC,
            'car_timeout_ms':    500,    # bundan eski bearing valid=0 gider
            # IR LED dik ucgeni → huni merkezinin NORMALIZE YONU (X/Z, Y/Z).
            # BIRIM UYARISI: irpos metre DEGIL, boyutsuz. Menzille carpmayi
            # otopilot kendi (EKF-fuzyonlu) yuksekligiyle yapiyor.
            # Yon hipotenusun ORTA noktasindan cikar (= huni merkezi).
            # Ikizkenar: iki dik kenar da 1.36 m -> hipotenus 1.92 m. Bu boyutta
            # ucgen ~2 m altinda kadraja SIGMAZ (1.92m @ Z=2m ≈ 615 px); pratik
            # calisma araligi ~2-15 m.
            'ir_enabled':        True,
            'camera_info_topic': '/camera/camera/infra1/camera_info',
            'ir_leg_a_m':        1.36,
            'ir_leg_b_m':        1.36,
            # Sentetik testle secildi (ros2 run lps_serial_sender test_ir_triangle):
            # mesru tespit 30 derece egimde bile fit=0.074, ESKENAR ucgen 0.157.
            # 0.25 iken eskenar ucgen kabul ediliyordu.
            'ir_tol_frac':       0.12,   # kenar basina ORTALAMA goreli hata tavani
            # Optik Z (Z = f/k, ucgenin piksel boyutundan) PAKETE GITMIYOR —
            # yanlis ucgen eslesmelerini elemek icin radar ile kiyaslanir ve
            # CSV'ye z_opt olarak yazilir. Dogrulama icin %10 dogruluk yeter,
            # optik Z bunu 40 derece egimde bile saglar.
            'ir_z_tol':          0.50,   # optik Z ile radar Z arasi izin verilen fark
            # MENZILI BU BELIRLER: LED'in piksel izi 1/Z^2 ile kuculur, tespiti
            # sinirlayan ilk sey genelde esik degil bu alan tabanidir. 1 px =
            # en genis menzil; tek piksel gurultusunu ucgen geometrisi eliyor.
            # Videodaki 'amin=Npx' bu tabana ne kadar yaklasildigini gosterir.
            'ir_min_area':       1.0,
            'ir_max_area':       400.0,
            # infra1'i AprilTag + optik akis da tuketiyor -> pozlama KISILAMAZ
            # (sahne kararir, ikisi de coker). Ayrim yuksek esikten gelir:
            # kameraya bakan aktif LED normal pozlamada da 255'e doygunlasir,
            # sahne doygunlasmaz. Video HUD'undaki blob= sayisi ile ayarlanir.
            'ir_min_threshold':  240.0,
            # Ham akis CSV logu — analiz icin, opsiyonel (varsayilan kapali).
            # 'flow_csv':     True,
            # IR CSV logu — ucus sonrasi analiz. record_dir'e ir_YYYYmmdd...csv
            # duser, HER kare yazilir (tespit olmayan kareler esik ayari icin
            # asil degerli veri). Ilk saha testlerinde ACIK tutulmasi onerilir.
            'ir_csv':        True,
        }],
    )

    return LaunchDescription([
        LogInfo(msg=f'LPS kayit oturumu #{record_session} -> {RECORD["record_dir"]}/*_s{record_session:04d}_*'),
        backend_arg,
        tag_family_arg,
        infra2_test_arg,
        enable_car_arg,
        car_record_arg,
        car_record_scale_arg,
        car_copy_hw_arg,
        car_model_arg,
        camera_fps_arg,
        infra2_equal_arg,
        serial_port_arg,
        fastdds_shm_arg,
        apriltag_every_n_arg,
        set_fastdds_profile,     # node'lardan ONCE: hepsi (realsense dahil) ayni profili alsin
        realsense_launch,
        TimerAction(period=3.0, actions=[disable_emitter]),
        TimerAction(period=3.5, actions=[apriltag_container]),   # sadece gpu backend
        TimerAction(period=4.0, actions=[cpu_apriltag]),         # sadece cpu backend
        TimerAction(period=5.5, actions=[serial_sender]),
        # infra2 test: ucus node'lari ayaga kalktiktan SONRA (sadece infra2_test)
        TimerAction(period=6.5, actions=[cpu_apriltag_infra2, infra2_recorder]),
        # arac tespiti en son (ilk acilista TensorRT motoru yoksa dakikalarca derler)
        TimerAction(period=7.5, actions=[car_detector]),
    ])
