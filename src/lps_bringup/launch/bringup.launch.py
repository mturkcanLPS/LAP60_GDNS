from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_infra1': 'true',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'depth_module.infra_profile': '1280x720x30',
            'depth_module.depth_profile': '1280x720x30',
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

    # 2) AprilTag container: mono→rgb dönüştürücü + AprilTag
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
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
                    'size': 0.984,
                    'tag_family': 'tag36h11',
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
            'num_levels':   3,
            'ema_alpha':    0.25,
            'debug':        True,   # periyodik loglar kapali
            # Ucus video kaydi: tag koseleri + akis HUD'u overlay'li .avi
            # (ucus sonrasi analiz icin). Kapatmak icin False.
            'record_video': True,
            'record_dir':   '/home/lapis/lps_records',
            'record_fps':   15.0,
            'record_scale': 0.5,
        }],
    )

    return LaunchDescription([
        realsense_launch,
        TimerAction(period=3.0, actions=[camera_tune]),
        TimerAction(period=3.5, actions=[apriltag_container]),
        TimerAction(period=5.5, actions=[serial_sender]),
    ])