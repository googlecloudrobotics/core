load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def cloud_robotics_repositories():
    # Eigen
    # Based on https://github.com/tensorflow/tensorflow/blob/master/third_party/eigen.BUILD
    _maybe(
        http_archive,
        name = "org_tuxfamily_eigen",
        build_file = "@cloud_robotics//third_party:eigen.BUILD",
        sha256 = "ca7beac153d4059c02c8fc59816c82d54ea47fe58365e8aded4082ded0b820c4",
        strip_prefix = "eigen-eigen-f3a22f35b044",
        urls = [
            "http://mirror.bazel.build/bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
            "https://bitbucket.org/eigen/eigen/get/f3a22f35b044.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "bazel_skylib",
        sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
            "https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_google_protobuf",
        sha256 = "bb1ddd8172b745cbdc75f06841bd9e7c9de0b3956397723d883423abfab8e176",
        strip_prefix = "protobuf-3.18.0",
        urls = [
            "https://mirror.bazel.build/github.com/protocolbuffers/protobuf/archive/v3.18.0.zip",
            "https://github.com/protocolbuffers/protobuf/archive/v3.18.0.zip",
        ],
    )

    # protos for the Google APIs
    _maybe(
        http_archive,
        name = "com_github_googleapis_googleapis",
        patch_args = ["-p1"],
        sha256 = "77216f166548374668a0a8ab1502acf3da6502affa826efca58cf6343b5550ed",
        strip_prefix = "googleapis-58ecdb1f0c0297975e68df5b45200a2d06f6d933",
        urls = [
            "https://mirror.bazel.build/github.com/googleapis/googleapis/archive/58ecdb1f0c0297975e68df5b45200a2d06f6d933.tar.gz",
            "https://github.com/googleapis/googleapis/archive/58ecdb1f0c0297975e68df5b45200a2d06f6d933.tar.gz",
        ],
    )

    # TODO(rodrigoq): rename to com_github_antonovvk_bazel_rules to match cartographer.
    _maybe(
        http_archive,
        name = "bazel_rules",
        sha256 = "2f5327a2dc9a0cc8ead93953a5d2ae2e0308aece685e46cc89c27538a7e9a73a",
        strip_prefix = "bazel_rules-c76e47ebe6f0a03b9dd99e245d5a0611813c36f9",
        urls = [
            "https://github.com/drigz/bazel_rules/archive/c76e47ebe6f0a03b9dd99e245d5a0611813c36f9.tar.gz",
        ],
    )

    # EmPy
    _maybe(
        http_archive,
        name = "empy_repo",
        build_file = "@cloud_robotics//third_party:empy.BUILD",
        sha256 = "9841e36dd26c7f69fe1005f9d9e078e41bdd50dd56fc77837ae390fb6af1aed7",
        strip_prefix = "empy-3.3.3",
        urls = [
            "https://mirror.bazel.build/www.alcyone.com/software/empy/empy-3.3.3.tar.gz",
            "http://www.alcyone.com/software/empy/empy-3.3.3.tar.gz",
        ],
    )

    # Support library for code generation for ROS messages (0.5.9)
    _maybe(
        http_archive,
        name = "com_github_ros_genmsg",
        build_file = "@cloud_robotics//third_party:genmsg.BUILD",
        sha256 = "cd0a2cdc1faa060fa96bbc91621d6534a99623cd1d5f2470a077153460537f62",
        strip_prefix = "genmsg-760e4f6819ff725780dd6fbf8086912009ce3d8c",
        urls = [
            "https://mirror.bazel.build/github.com/ros/genmsg/archive/760e4f6819ff725780dd6fbf8086912009ce3d8c.tar.gz",
            "https://github.com/ros/genmsg/archive/760e4f6819ff725780dd6fbf8086912009ce3d8c.tar.gz",
        ],
    )

    # Code generation of ROS messages for C++.
    _maybe(
        http_archive,
        name = "com_github_ros_gencpp",
        build_file = "@cloud_robotics//third_party:gencpp.BUILD",
        sha256 = "138bd77f2558f020bee260aab3446bbade24c591ce57da7be78b63c10dee67e9",
        strip_prefix = "gencpp-90a850954f7dec3d7274b4fc36d35b1fe6676b22",
        urls = [
            "https://mirror.bazel.build/github.com/ros/gencpp/archive/90a850954f7dec3d7274b4fc36d35b1fe6676b22.tar.gz",
            "https://github.com/ros/gencpp/archive/90a850954f7dec3d7274b4fc36d35b1fe6676b22.tar.gz",
        ],
    )

    # Code generation of ROS messages for Python.
    _maybe(
        http_archive,
        name = "com_github_ros_genpy",
        build_file = "@cloud_robotics//third_party:genpy.BUILD",
        sha256 = "452bb938d2e7ff4e82b54a89bc0fef2b3d7da22245d40f26275c40baf461eae8",
        strip_prefix = "genpy-32c78d763b27232a9e971da8054e0215b41daa1b",
        urls = [
            "https://mirror.bazel.build/github.com/ros/genpy/archive/32c78d763b27232a9e971da8054e0215b41daa1b.tar.gz",
            "https://github.com/ros/genpy/archive/32c78d763b27232a9e971da8054e0215b41daa1b.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_std_msgs",
        build_file = "@cloud_robotics//third_party:std_msgs.BUILD",
        sha256 = "d39866a32e9d4f4d29276bd1ed7943cb52933c43531910d0c759580d749f9995",
        strip_prefix = "std_msgs-474568b32881c81f6fb962a1b45a7d60c4db9255",
        urls = [
            "https://mirror.bazel.build/github.com/ros/std_msgs/archive/474568b32881c81f6fb962a1b45a7d60c4db9255.tar.gz",
            "https://github.com/ros/std_msgs/archive/474568b32881c81f6fb962a1b45a7d60c4db9255.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_common_msgs",
        build_file = "@cloud_robotics//third_party:common_msgs.BUILD",
        sha256 = "f30449e96ad77682ab94741c57536f26790f83cc4f31855a019e7649f229868b",
        strip_prefix = "common_msgs-25613ec79003d8caf0541d7ab36820ed44ebef93",
        urls = [
            "https://mirror.bazel.build/github.com/ros/common_msgs/archive/25613ec79003d8caf0541d7ab36820ed44ebef93.tar.gz",
            "https://github.com/ros/common_msgs/archive/25613ec79003d8caf0541d7ab36820ed44ebef93.tar.gz",
        ],
    )

    # TODO(rodrigoq): update to latest version - will need to replace CMake's
    # generate_export_header, eg with
    # https://github.com/RobotLocomotion/drake/blob/63f160f3f4bc8b30088e1191253c65fde7b8eeee/tools/generate_export_header.bzl
    _maybe(
        http_archive,
        name = "com_github_ros_console_bridge",
        build_file = "@cloud_robotics//third_party:console_bridge.BUILD",
        sha256 = "4da12433fddf93187a67654f0593f77f4fa94a83b9a6638de29a686cdcc769c9",
        strip_prefix = "console_bridge-148df7e841b91da488ec3bb4abd295a2bccdb728",
        urls = [
            "https://mirror.bazel.build/github.com/ros/console_bridge/archive/148df7e841b91da488ec3bb4abd295a2bccdb728.tar.gz",
            "https://github.com/ros/console_bridge/archive/148df7e841b91da488ec3bb4abd295a2bccdb728.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_roscpp_core",
        build_file = "@cloud_robotics//third_party:roscpp_core.BUILD",
        sha256 = "0684e503a4382379a6e9ba4e0885df1791faef01e2aa7928ec292d45e4642cad",
        strip_prefix = "roscpp_core-3b362ac2369158faaaec9a5734bdd08cf341a2d9",
        urls = [
            "https://mirror.bazel.build/github.com/ros/roscpp_core/archive/3b362ac2369158faaaec9a5734bdd08cf341a2d9.tar.gz",
            "https://github.com/ros/roscpp_core/archive/3b362ac2369158faaaec9a5734bdd08cf341a2d9.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_infrastructure_rospkg",
        build_file = "@cloud_robotics//third_party:rospkg.BUILD",
        sha256 = "e4ab253fdf43527a651494da23302de9f4e90e360597efb14b899b87573e625a",
        strip_prefix = "rospkg-bffd0aed87fcbf495361289b850be255bd4e1edd",
        urls = [
            "https://mirror.bazel.build/github.com/ros-infrastructure/rospkg/archive/bffd0aed87fcbf495361289b850be255bd4e1edd.tar.gz",
            "https://github.com/ros-infrastructure/rospkg/archive/bffd0aed87fcbf495361289b850be255bd4e1edd.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_infrastructure_catkin_pkg",
        build_file = "@cloud_robotics//third_party:catkin_pkg.BUILD",
        sha256 = "f93f2f66d714b58c51b6854b3aac2fe1cbc46db0e51597bbf5853bb00e04d8eb",
        strip_prefix = "catkin_pkg-40dbc2dd3ae8b2787cfb2a59776a944d3f1e4655",
        urls = [
            "https://mirror.bazel.build/github.com/ros-infrastructure/catkin_pkg/archive/40dbc2dd3ae8b2787cfb2a59776a944d3f1e4655.tar.gz",
            "https://github.com/ros-infrastructure/catkin_pkg/archive/40dbc2dd3ae8b2787cfb2a59776a944d3f1e4655.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_catkin",
        build_file = "@cloud_robotics//third_party:catkin.BUILD",
        sha256 = "1fd0affb22ec3ae63849d231f681393966f6d0dd83c51bd6a5fe18239496f7a4",
        strip_prefix = "catkin-1bb11b704385f6ea7a8a055a6ee3df1c67b3df10",
        urls = [
            "https://mirror.bazel.build/github.com/ros/catkin/archive/1bb11b704385f6ea7a8a055a6ee3df1c67b3df10.tar.gz",
            "https://github.com/ros/catkin/archive/1bb11b704385f6ea7a8a055a6ee3df1c67b3df10.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_ros_comm_msgs",
        build_file = "@cloud_robotics//third_party:ros_comm_msgs.BUILD",
        sha256 = "845b32e6ca28bcf3db7c6a29143853d51856280f1f103e913b94401c1d1a15e2",
        strip_prefix = "ros_comm_msgs-bfb8533fd6c6e959c71f0d2a9669baeff2dac1ad",
        urls = [
            "https://mirror.bazel.build/github.com/ros/ros_comm_msgs/archive/bfb8533fd6c6e959c71f0d2a9669baeff2dac1ad.tar.gz",
            "https://github.com/ros/ros_comm_msgs/archive/bfb8533fd6c6e959c71f0d2a9669baeff2dac1ad.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_ros_comm",
        build_file = "@cloud_robotics//third_party:ros_comm.BUILD",
        sha256 = "21ec5c81910e82bcfd3ea596b08940d25f256bce5b4c3622e5b4bdd3e7e32595",
        strip_prefix = "ros_comm-ba1413054cebb28960b25d887c25049eb87a9ad1",
        urls = [
            "https://mirror.bazel.build/github.com/ros/ros_comm/archive/ba1413054cebb28960b25d887c25049eb87a9ad1.tar.gz",
            "https://github.com/ros/ros_comm/archive/ba1413054cebb28960b25d887c25049eb87a9ad1.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_geometry2",
        build_file = "@cloud_robotics//third_party:geometry2.BUILD",
        sha256 = "83400a0d160179d275257ccce801219d2c64b94f24658c67bc899d17756018ad",
        strip_prefix = "geometry2-ef31bb6d9af9e253e2715e2ff83c1242e0897cc3",
        urls = [
            "https://mirror.bazel.build/github.com/ros/geometry2/archive/ef31bb6d9af9e253e2715e2ff83c1242e0897cc3.tar.gz",
            "https://github.com/ros/geometry2/archive/ef31bb6d9af9e253e2715e2ff83c1242e0897cc3.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_actionlib",
        build_file = "@cloud_robotics//third_party:actionlib.BUILD",
        sha256 = "02a3102393ec9949b61cef49a6374883c777a38421bd0824237d2085fcbdf49b",
        strip_prefix = "actionlib-31b5d74eaa102dbc39243af7eca2b4c6fcc53786",
        urls = [
            "https://mirror.bazel.build/github.com/ros/actionlib/archive/31b5d74eaa102dbc39243af7eca2b4c6fcc53786.tar.gz",
            "https://github.com/ros/actionlib/archive/31b5d74eaa102dbc39243af7eca2b4c6fcc53786.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_robot_model",
        build_file = "@cloud_robotics//third_party:robot_model.BUILD",
        sha256 = "3129c5e494523d0e8c0241541e5f6f25f2cc2b5e06724cd5cb9fb9e697afe0e6",
        strip_prefix = "robot_model-242f07614a5d8265edee43ba35f3d8b77b16c136",
        urls = [
            "https://mirror.bazel.build/github.com/ros/robot_model/archive/242f07614a5d8265edee43ba35f3d8b77b16c136.tar.gz",
            "https://github.com/ros/robot_model/archive/242f07614a5d8265edee43ba35f3d8b77b16c136.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_ros",
        build_file = "@cloud_robotics//third_party:ros.BUILD",
        sha256 = "758cd22d2f56ddf5390706d3adac9a99de326077d4bc84f073fb973740471c86",
        strip_prefix = "ros-782d333f26f6f6f020412b3f56669ecace690d81",
        urls = [
            "https://mirror.bazel.build/github.com/ros/ros/archive/782d333f26f6f6f020412b3f56669ecace690d81.tar.gz",
            "https://github.com/ros/ros/archive/782d333f26f6f6f020412b3f56669ecace690d81.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_class_loader",
        build_file = "@cloud_robotics//third_party:class_loader.BUILD",
        sha256 = "c57a9e7ba3cf6d1e58024d09901aa773978bc161cf96295dba6df50e96565013",
        strip_prefix = "class_loader-e986135b307c96ab7256a8553dccf8ebdf1a6423",
        urls = [
            "https://mirror.bazel.build/github.com/ros/class_loader/archive/e986135b307c96ab7256a8553dccf8ebdf1a6423.tar.gz",
            "https://github.com/ros/class_loader/archive/e986135b307c96ab7256a8553dccf8ebdf1a6423.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_pluginlib",
        build_file = "@cloud_robotics//third_party:pluginlib.BUILD",
        sha256 = "34c52ec8876cc1097b61a8a73c7a9d5f9aa4d895b0d9adf41bc8756d7e439485",
        strip_prefix = "pluginlib-f0813984256b4a2b3221df7981ecc086312429ba",
        urls = [
            "https://mirror.bazel.build/github.com/ros/pluginlib/archive/f0813984256b4a2b3221df7981ecc086312429ba.tar.gz",
            "https://github.com/ros/pluginlib/archive/f0813984256b4a2b3221df7981ecc086312429ba.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_urdfdom",
        build_file = "@cloud_robotics//third_party:urdfdom.BUILD",
        sha256 = "cbfdc284f1d08c3893ce1087c0d93d08ee4367996ee43b80fffd0bc9da3cad82",
        strip_prefix = "urdfdom-8b27e0b6fbddbb4cc670156961bd52b4d5ea632b",
        urls = [
            "https://mirror.bazel.build/github.com/ros/urdfdom/archive/8b27e0b6fbddbb4cc670156961bd52b4d5ea632b.tar.gz",
            "https://github.com/ros/urdfdom/archive/8b27e0b6fbddbb4cc670156961bd52b4d5ea632b.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_urdfdom_headers",
        build_file = "@cloud_robotics//third_party:urdfdom_headers.BUILD",
        sha256 = "98ad8e98bcac10c2ae78c464ac60c0dd0fb8c6c5b5ef1164193268fc60c41c65",
        strip_prefix = "urdfdom_headers-65f3a6c4e4e98898b146876a1e53d834b9a96a60",
        urls = [
            "https://mirror.bazel.build/github.com/ros/urdfdom_headers/archive/65f3a6c4e4e98898b146876a1e53d834b9a96a60.tar.gz",
            "https://github.com/ros/urdfdom_headers/archive/65f3a6c4e4e98898b146876a1e53d834b9a96a60.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_ros_planning_navigation_msgs",
        build_file = "@cloud_robotics//third_party:navigation_msgs.BUILD",
        sha256 = "3d7a19f04b0bcdc59b8e4180333cd6ae01131c209b63150014acae762827dde3",
        strip_prefix = "navigation_msgs-1eae598d3626f35c0a0721cbdc73209b734a863e",
        urls = [
            "https://mirror.bazel.build/github.com/ros-planning/navigation_msgs/archive/1eae598d3626f35c0a0721cbdc73209b734a863e.tar.gz",
            "https://github.com/ros-planning/navigation_msgs/archive/1eae598d3626f35c0a0721cbdc73209b734a863e.tar.gz",
        ],
    )

    # tinyxml 2.6.2 (mirrored on GitHub to avoid depending on SourceForge too)
    _maybe(
        http_archive,
        name = "com_github_icebreaker_tinyxml",
        build_file = "@cloud_robotics//third_party:tinyxml.BUILD",
        sha256 = "ea7db1f158f08dd0e4927112caaa620a341fea2e2717c99c5a807eca874f017d",
        strip_prefix = "TinyXML-b3e5aabf9c0272fdfd05487b5ea12eaf4522713d",
        urls = [
            "https://mirror.bazel.build/github.com/icebreaker/TinyXML/archive/b3e5aabf9c0272fdfd05487b5ea12eaf4522713d.tar.gz",
            "https://github.com/icebreaker/TinyXML/archive/b3e5aabf9c0272fdfd05487b5ea12eaf4522713d.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_leethomason_tinyxml2",
        build_file = "@cloud_robotics//third_party:tinyxml2.BUILD",
        sha256 = "a381729e32b6c2916a23544c04f342682d38b3f6e6c0cad3c25e900c3a7ef1a6",
        strip_prefix = "tinyxml2-7.0.1",
        urls = [
            "https://github.com/leethomason/tinyxml2/archive/7.0.1.tar.gz",
        ],
    )

    # zlib
    _maybe(
        http_archive,
        name = "net_zlib_zlib",
        build_file = "@cloud_robotics//third_party:zlib.BUILD",
        sha256 = "6d4d6640ca3121620995ee255945161821218752b551a1a180f4215f7d124d45",
        strip_prefix = "zlib-cacf7f1d4e3d44d871b605da3b647f07d718623f",
        urls = [
            "https://mirror.bazel.build/github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
            "https://github.com/madler/zlib/archive/cacf7f1d4e3d44d871b605da3b647f07d718623f.tar.gz",
        ],
    )

    # bz2
    _maybe(
        http_archive,
        name = "org_bzip_bzip2",
        build_file = "@cloud_robotics//third_party:bzip2.BUILD",
        sha256 = "a2848f34fcd5d6cf47def00461fcb528a0484d8edef8208d6d2e2909dc61d9cd",
        strip_prefix = "bzip2-1.0.6",
        urls = [
            "https://mirror.bazel.build/www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz",
            "http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_cyan4973_xxhash",
        build_file = "@cloud_robotics//third_party:xxhash.BUILD",
        sha256 = "d8c739ec666ac2af983a61dc932aaa2a8873df974d333a9922d472a121f2106e",
        strip_prefix = "xxHash-0.6.3",
        urls = ["https://github.com/Cyan4973/xxHash/archive/v0.6.3.tar.gz"],
    )

    _maybe(
        http_archive,
        name = "com_github_lz4_lz4",
        build_file = "@cloud_robotics//third_party:lz4.BUILD",
        sha256 = "2ca482ea7a9bb103603108b5a7510b7592b90158c151ff50a28f1ca8389fccf6",
        strip_prefix = "lz4-1.8.0",
        urls = ["https://github.com/lz4/lz4/archive/v1.8.0.tar.gz"],
    )

    # POCO
    _maybe(
        http_archive,
        name = "org_pocoproject_poco",
        build_file = "@cloud_robotics//third_party:poco.BUILD",
        sha256 = "40743cf18fadea6992e0ad7f668a75d46f08364a7f3ff748420fa080bbaaa3d1",
        strip_prefix = "poco-1.7.8p3",
        urls = [
            "https://mirror.bazel.build/pocoproject.org/releases/poco-1.7.8/poco-1.7.8p3.tar.gz",
            "https://pocoproject.org/releases/poco-1.7.8/poco-1.7.8p3.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_gt_rail_rosauth",
        build_file = "@cloud_robotics//third_party:rosauth.BUILD",
        sha256 = "c85c4163c94d20bce9864180785210bfe3dfd110114f18212bd711d1be87c48e",
        strip_prefix = "rosauth-0.1.7",
        urls = ["https://github.com/GT-RAIL/rosauth/archive/0.1.7.tar.gz"],
    )

    # Boost
    SOURCEFORGE_MIRRORS = ["phoenixnap", "newcontinuum", "cfhcable", "superb-sea2", "cytranet", "iweb", "gigenet", "ayera", "astuteinternet", "pilotfiber", "svwh"]
    _maybe(
        http_archive,
        name = "boost",
        urls = [
            "https://%s.dl.sourceforge.net/project/boost/boost/1.63.0/boost_1_63_0.tar.gz" % m
            for m in SOURCEFORGE_MIRRORS
        ],
        build_file = "@com_github_nelhage_rules_boost//:BUILD.boost",
        strip_prefix = "boost_1_63_0",
        sha256 = "fe34a4e119798e10b8cc9e565b3b0284e9fd3977ec8a1b19586ad1dec397088b",
    )

    _maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        sha256 = "89ea32a8adb7521d06447c87bfb4517ff60f3834bd462c17c4b8f79047460410",
        strip_prefix = "rules_boost-f0ecfe836da225d4a396ef5f604e885ce70636fb",
        urls = [
            "https://github.com/nelhage/rules_boost/archive/f0ecfe836da225d4a396ef5f604e885ce70636fb.tar.gz",
        ],
    )

    # Binary build of the helm package manager for Kubernetes
    _maybe(
        http_archive,
        name = "kubernetes_helm",
        build_file = "@cloud_robotics//third_party/helm2:BUILD.bazel",
        sha256 = "f3bec3c7c55f6a9eb9e6586b8c503f370af92fe987fcbf741f37707606d70296",
        strip_prefix = "linux-amd64",
        urls = [
            "https://get.helm.sh/helm-v2.17.0-linux-amd64.tar.gz",
        ],
    )
    _maybe(
        http_archive,
        name = "kubernetes_helm3",
        build_file = "@cloud_robotics//third_party/helm3:BUILD.bazel",
        sha256 = "1484ffb0c7a608d8069470f48b88d729e88c41a1b6602f145231e8ea7b43b50a",
        strip_prefix = "linux-amd64",
        urls = [
            "https://get.helm.sh/helm-v3.9.0-linux-amd64.tar.gz",
        ],
    )

    # Binary build of the terraform infrastructure management tool
    _maybe(
        http_archive,
        name = "hashicorp_terraform",
        build_file = "@cloud_robotics//third_party:terraform.BUILD",
        sha256 = "e5eeba803bc7d8d0cae7ef04ba7c3541c0abd8f9e934a5e3297bf738b31c5c6d",
        urls = [
            "https://releases.hashicorp.com/terraform/0.12.31/terraform_0.12.31_linux_amd64.zip",
        ],
    )

    # Rules for building and handling Docker images with Bazel and define base images
    _maybe(
        http_archive,
        name = "io_bazel_rules_docker",
        sha256 = "85ffff62a4c22a74dbd98d05da6cf40f497344b3dbf1e1ab0a37ab2a1a6ca014",
        strip_prefix = "rules_docker-0.23.0",
        urls = ["https://github.com/bazelbuild/rules_docker/releases/download/v0.23.0/rules_docker-v0.23.0.tar.gz"],
    )

    # Go rules and proto support
    _maybe(
        http_archive,
        name = "io_bazel_rules_go",
        sha256 = "2b1641428dff9018f9e85c0384f03ec6c10660d935b750e3fa1492a281a53b0f",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.29.0/rules_go-v0.29.0.zip",
            "https://github.com/bazelbuild/rules_go/releases/download/v0.29.0/rules_go-v0.29.0.zip",
        ],
    )

    _maybe(
        http_archive,
        name = "bazel_gazelle",
        sha256 = "de69a09dc70417580aabf20a28619bb3ef60d038470c7cf8442fafcf627c21cb",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
            "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
        ],
    )

    _maybe(
        http_archive,
        name = "com_github_bazelbuild_buildtools",
        sha256 = "ae34c344514e08c23e90da0e2d6cb700fcd28e80c02e23e4d5715dddcb42f7b3",
        strip_prefix = "buildtools-4.2.2",
        urls = [
            "https://github.com/bazelbuild/buildtools/archive/4.2.2.tar.gz",
        ],
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
