from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction, ExecuteProcess,
                            DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration,
                                  PythonExpression)
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'true',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'depth_module.infra_profile': '1280x720x30',
            'depth_module.depth_profile': '1280x720x30',
            'rgb_camera.color_profile': '1280x720x15', 
        }.items()
    )

    # Kamera ayar WATCHDOG'u — rs_launch'in declared arg listesinde olmayan
    # opsiyonlari OTURUM BOYUNCA 10 sn'de bir dogrular, sapmissa geri basar:
    #  - emitter_enabled 0        : dot pattern optik akisi bozar (kamerayla
    #                               birlikte hareket eder, akisi sifira ceker)
    #  - global_time_enabled false: image stamp'leri KAMERANIN donanim
    #      saatinden gelsin (host'a hizalama YOK). 26_07_19 ucusu: sahada
    #      NTP, bayat saati ucus ORTASINDA +2g3.5s step atti; global-time
    #      kestirimcisi cildirip stamp'leri 18s→2845→48770→252816→185403s
    #      gezdirdi, otopilot dt'leri coplendi. Cihaz saati step gormez.
    #      (Node tarafinda ayrica TS-GUARD var — cift emniyet.)
    #  - auto_exposure_limit 2000 : AE acik kalir ama pozlama 2 ms'i asamaz
    #      (gunesli sahnede saturasyonu keser, motion blur'u sinirlar →
    #       AprilTag tespiti hareket halinde yasar).
    #      NOT: 26_07_14 gunku "2000 yetmedi → 1000 yap" denemeleri KIRIK
    #      ON CAMLI kamerayla yapildi — catlak, gunesi goruntuye perde gibi
    #      sacip olcumleri kirletti (sol-alt sabit lob, zamansal ortalama
    #      analiziyle kanitlandi). 1000us'un tek net etkisi gain'in ikiye
    #      katlanip golge gurultusunun %62 artmasiydi. Saglam camla taban
    #      deger 2000; yeni referans ucusuyla dogrulanacak.
    # Neden watchdog (tek seferlik retry degil):
    #  1) sahada sogut acilista kamera paramlari gec ilan edilebiliyor
    #     (26_07_14_T6: 11 sn'lik retry penceresi kacti, ucus limitsiz gitti)
    #  2) ucus sirasinda USB kopup gelirse wrapper varsayilana doner —
    #     watchdog ayari sessiz kayiptan kurtarip yeniden uygular.
    # Log disiplini: sadece durum DEGISIMINDE yazar (spam yok).
    # Kalkis oncesi "[CAM-WD] TUM AYARLAR AKTIF" satirini gormeden ucma!
    # Durum satirlari hem ekrana hem /home/lapis/lps_records/camwd.log'a
    # yazilir (ucus sirasinda terminal gorulemiyor; ucus sonrasi denetim
    # icin kalici iz). Not: watchdog manuel "ros2 param set" denemelerini
    # 10 sn icinde geri ezer — A/B testi icin buradaki degeri degistir.
    camera_tune = ExecuteProcess(
        cmd=['bash', '-c',
             'source /opt/ros/humble/setup.bash; '
             'N=/camera/camera; '
             'WDLOG=/home/lapis/lps_records/camwd.log; '
             'log() { echo "$(date "+%F %T") $*" | tee -a $WDLOG; }; '
             'log "[CAM-WD] watchdog basladi (hedef: emitter=0, AE limit=2000us, global_time=false)"; '
             'cas() { '                                 # check-and-set
             '  cur=$(timeout 8 ros2 param get $N "$1" 2>/dev/null); '
             '  if echo "$cur" | grep -q "value is: $3"; then return 0; fi; '
             '  if timeout 8 ros2 param set $N "$1" "$2" 2>&1 '
             '     | grep -q successful; then '
             '    log "[CAM-WD] $1=$2 UYGULANDI"; return 0; '
             '  fi; '
             '  return 1; '
             '}; '
             'prev=-1; '
             'while true; do '
             '  ok=1; '
             '  cas depth_module.emitter_enabled 0 "0" || ok=0; '
             '  cas depth_module.auto_exposure_limit_toggle true "[Tt]rue" || ok=0; '
             '  cas depth_module.auto_exposure_limit 2000 "2000" || ok=0; '
             '  cas depth_module.global_time_enabled false "[Ff]alse" || ok=0; '
             '  if [ "$ok" != "$prev" ]; then '
             '    if [ "$ok" = "1" ]; then '
             '      log "[CAM-WD] TUM AYARLAR AKTIF"; '
             '    else '
             '      log "[CAM-WD] KAMERA/PARAM ULASILAMIYOR — denemeye devam"; '
             '    fi; '
             '    prev=$ok; '
             '  fi; '
             '  sleep 10; '
             'done'],
        output='screen',
    )

    # 2) AprilTag container (GPU/Isaac): mono→rgb + AprilTag.
    #    SADECE apriltag_backend:=gpu iken calisir (varsayilan cpu).
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
          
            ComposableNode(
                package='isaac_ros_apriltag',
                plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
                name='apriltag',
                remappings=[
                    ('image', '/camera/camera/infra1/image_rgb'),
                    ('camera_info', '/camera/camera/infra1/camera_info'),
                ],
                parameters=[{
                    # tag25h9 huni duvar buyuk tag siyah kare = 250mm. GPU yolu
                    # aktif degil (varsayilan CPU) ama tutarli olsun diye guncel.
                    'size': 0.25,
                    'tag_family': 'tag25h9',
                    'max_tags': 64,
                    'tile_size': 4,
                }],
            ),
        ],
    )

   
    serial_sender = Node(
        package='lps_serial_sender',
        executable='serial_sender_node',
        name='serial_sender',
        output='screen',
        parameters=[{
         
            'serial_port':  '/dev/ttyTHS1',
            'baud_rate':    115200,
            'send_rate_hz': 40.0,
            'tag_topic':    '/tag_detections',
            'image_topic':  '/camera/camera/infra1/image_rect_raw',
            'grid_size':    4,
            # Piramit seviyesi = izlenebilir MAKSIMUM kayma (her seviye 2 kat).
            # 3 -> 32 px'e kadar, 4 -> 64 px. Kirilma noktasinin ALTINDA seviye
            # sayisinin dogruluga etkisi SIFIR (olculdu: 0.3-16 px araliginda
            # 2/3/4 ayni sonucu veriyor), yani 4 yapmak bedavaya tavan yukseltir.
            # Gercek ucus kareleriyle dogrulandi (faz korelasyonu hakemligiyle):
            # lv3'un kacirdigi 16-30 px'lik hareketleri (Q=10 basip veriyi
            # atiyordu) lv4 dogru olctu (Q=38-62). Hat kamera hizina
            # yetisemedigi icin kareler atlaniyor, bu da efektif kaymayi
            # katliyor — pay birakmak sart.
            'num_levels':   4,
            'ema_alpha':    0.25,
            'debug':        True,   # periyodik loglar kapali
            # Ucus video kaydi: tag koseleri + akis HUD'u overlay'li .avi
            # (ucus sonrasi analiz icin). Kapatmak icin False.
            'record_video': True,
            # Ham optik akis CSV'si (EMA ONCESI kare-basi olcum) — analiz icin.
            # record_dir'e flow_YYYYmmdd_HHMMSS.csv olarak duser.
            'flow_csv':     False,   # artik bakilmiyor -> kapali (CPU/disk yormasin)
            # Huni kalibrasyon CSV'si — her tespit edilen tag icin HAM poz
            # (Tpos + quaternion). record_dir'e funnel_YYYYmmdd_HHMMSS.csv.
            'funnel_csv':   True,
            'record_dir':   '/home/lapis/lps_records',
            'record_fps':   15.0,
            'record_scale': 0.5,
            # TEST: car_detector bearing'ini avgPos'a enjekte et (default False).
            # `car_avg_inject:=true` ile ac -> avgPos = car bearing (funnel EZILIR).
            'car_avg_inject': ParameterValue(
                LaunchConfiguration('car_avg_inject'), value_type=bool),
        }],
    )

    # === AprilTag arka ucu secimi ===
    #   apriltag_backend:=cpu  (VARSAYILAN) -> CPU (libapriltag), /tag_detections
    #   apriltag_backend:=gpu               -> Isaac GPU container, /tag_detections
    # Ikisi de tag25h9 kullanir ve AYNI topic'e (/tag_detections) yayinlar;
    # serial_sender degismeden hangisi acilirsa onu tuketir.
    # CPU'nun Isaac'a ustunlugu 26_08_13 A/B'de olculdu: uzak/egik sahnede
    # Isaac tag 20/24'u tamamen kacirdi (0), CPU 874/551 yakaladi (quad_decimate=1
    # tam cozunurluk + decode_sharpening sayesinde). Bu yuzden varsayilan CPU.
    backend_arg = DeclareLaunchArgument('apriltag_backend', default_value='cpu')
    # tag_family: `tag_family:=tagCircle21h7` ile aile degistirip dene.
    # Desteklenen: tag36h11/tag36h10/tag25h9/tag16h5/tagCircle21h7/tagCircle49h12/
    #              tagCustom48h12/tagStandard41h12/tagStandard52h13
    # Huni tag25h9 (gecis) -> default bu. Baska aile icin `tag_family:=...` ile ez.
    tag_family_arg = DeclareLaunchArgument('tag_family', default_value='tag25h9')
    # TEST: car_detector bearing'ini serial_sender avgPos'a enjekte et (default false).
    car_inject_arg = DeclareLaunchArgument('car_avg_inject', default_value='true')
    # car_detector'i bringup'la beraber ac (default ACIK — bir daha unutma!).
    # Kapatmak icin: enable_car:=false
    enable_car_arg = DeclareLaunchArgument('enable_car', default_value='true')
    cpu_apriltag = Node(
        package='lps_serial_sender', executable='cpu_apriltag_node',
        name='cpu_apriltag', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('apriltag_backend'), "' == 'cpu'"])),
        parameters=[{
            'image_topic':       '/camera/camera/infra1/image_rect_raw',
            'camera_info_topic': '/camera/camera/infra1/camera_info',
            'output_topic':      '/tag_detections',   # asil topic (serial_sender abone)
            'tag_family':        LaunchConfiguration('tag_family'),  # arg ile degistir
            'size':              0.25,    # tag25h9 huni duvar buyuk tag siyah kare = 250mm
            'quad_decimate':     1.0,          # tam cozunurluk (kucuk/uzak tag)
            'quad_sigma':        0.0,
            'decode_sharpening': 1.0,         # bulanik/egik tag icin artir
            'refine_edges':      True,
            'nthreads':          4,   # 6 cekirdekten 4'u: yogun/ucus karelerinde drop'u azaltir
                                       # (33ms taban seri islemden; thread onu dusurmez). TX core5'te.
        }],
    )

    # 4) EO araç tespiti (DeepStream YOLO + NvDCF) — color stream'den, kilit + bearing
    #    -> /car_detection/bearing. EO video kaydini da alir (overlay'li .mkv).
    #    enable_car:=false ile kapat.
    car_detector = Node(
        package='car_detector', executable='car_detector_node',
        name='car_detector', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_car')),
        parameters=[{
            'nvinfer_config':  '/home/lapis/DeepStream-Yolo/config_infer_primary_yoloV8.txt',
            'vehicle_classes': [3, 5],       # VisDrone: car + truck (Hilux)
            'mux_width':       1280,
            'mux_height':      720,
            'record_video':    True,         # EO .mkv kaydi (overlay gomulu)
            'record_dir':      '/home/lapis/lps_records',
            # RealSense D435i Color 1280x720 fabrika intrinsics (SABIT — camera_info
            # beklemeden bearing baslar). Distortion sifir. Cozunurluk degisirse
            # rs-enumerate-devices -c ile yeniden al.
            'fx': 909.501, 'fy': 909.527, 'cx': 643.236, 'cy': 359.794,
        }],
    )

    return LaunchDescription([
        backend_arg,
        tag_family_arg,
        car_inject_arg,
        enable_car_arg,
        realsense_launch,
        TimerAction(period=3.0, actions=[camera_tune]),
        TimerAction(period=3.5, actions=[apriltag_container]),  # sadece gpu backend'de
        TimerAction(period=4.0, actions=[cpu_apriltag]),        # sadece cpu backend'de
        TimerAction(period=5.5, actions=[serial_sender]),
        TimerAction(period=6.5, actions=[car_detector]),        # enable_car:=false ile kapat
    ])