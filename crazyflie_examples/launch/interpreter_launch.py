from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from huggingface_hub import hf_hub_download


def generate_launch_description():

    def run_whisper(context: LaunchContext, repo, file):
        repo = str(context.perform_substitution(repo))
        file = str(context.perform_substitution(file))

        return Node(
            package="whisper_ros",
            executable="whisper_node",
            name="whisper_node",
            namespace="whisper",
            parameters=[{
                "model": LaunchConfiguration("model", default=hf_hub_download(repo_id=repo, filename=file, force_download=False)),
                "openvino_encode_device": LaunchConfiguration("openvino_encode_device", default="CPU"),

                "n_threads": LaunchConfiguration("n_threads", default=4),
                "n_max_text_ctx": LaunchConfiguration("n_max_text_ctx", default=16384),
                "offset_ms": LaunchConfiguration("offset_ms", default=0),
                "duration_ms": LaunchConfiguration("duration_ms", default=0),

                "translate": LaunchConfiguration("translate", default=False),
                "no_context": LaunchConfiguration("no_context", default=True),
                "single_segment": LaunchConfiguration("single_segment", default=True),
                "print_special": LaunchConfiguration("print_special", default=False),
                "print_progress": LaunchConfiguration("print_progress", default=False),
                "print_realtime": LaunchConfiguration("print_realtime", default=False),
                "print_timestamps": LaunchConfiguration("print_timestamps", default=False),

                "token_timestamps": LaunchConfiguration("token_timestamps", default=False),
                "thold_pt": LaunchConfiguration("thold_pt", default=0.01),
                "thold_ptsum": LaunchConfiguration("thold_ptsum", default=0.01),
                "max_len": LaunchConfiguration("max_len", default=0),
                "split_on_word": LaunchConfiguration("split_on_word", default=False),
                "max_tokens": LaunchConfiguration("max_tokens", default=32),

                "tinydiarize": LaunchConfiguration("tinydiarize", default=False),
                "speed_up": LaunchConfiguration("speed_up", default=False),
                "audio_ctx": LaunchConfiguration("audio_ctx", default=0),

                "language": LaunchConfiguration("language", default="ja"),
                "detect_language": LaunchConfiguration("detect_language", default=False),

                "suppress_blank": LaunchConfiguration("suppress_blank", default=True),
                "suppress_non_speech_tokens": LaunchConfiguration("suppress_non_speech_tokens", default=False),

                "temperature": LaunchConfiguration("temperature", default=0.00),
                "max_initial_ts": LaunchConfiguration("max_initial_ts", default=1.00),
                "length_penalty": LaunchConfiguration("length_penalty", default=-1.00),

                "temperature_inc": LaunchConfiguration("temperature_inc", default=0.40),
                "entropy_thold": LaunchConfiguration("entropy_thold", default=2.40),
                "logprob_thold": LaunchConfiguration("logprob_thold", default=-1.00),
                "no_speech_thold": LaunchConfiguration("no_speech_thold", default=0.60),
            }]
        ),

    model_repo = LaunchConfiguration("model_repo")
    model_repo_cmd = DeclareLaunchArgument(
        "model_repo",
        default_value="ggerganov/whisper.cpp",
        description="Hugging Face model repo")

    model_filename = LaunchConfiguration("model_filename")
    model_filename_cmd = DeclareLaunchArgument(
        "model_filename",
        default_value="ggml-small.bin",
        description="Hugging Face model filename")

    return LaunchDescription([
        model_repo_cmd,
        model_filename_cmd,
        OpaqueFunction(function=run_whisper, args=[
                       model_repo, model_filename]),

        Node(
            package="whisper_ros",
            executable="silero_vad_node",
            name="silero_vad_node",
            namespace="whisper",
            parameters=[{
                "enabled": LaunchConfiguration("enabled", default=False),
            }],
            remappings=[("audio", "/audio/in")]
        ),
        Node(
            package="whisper_ros",
            executable="whisper_manager_node",
            name="whisper_manager_node",
            namespace="whisper",
        ),
        Node(
            package="whisper_ros",
            executable="esp_cam_node",
            name="esp_cam_node",
            namespace="whisper",
        ),
        Node(
            package="audio_common",
            executable="audio_capturer_node",
            name="audio_capturer_node",
            parameters=[{
                "format": LaunchConfiguration("channels", default=1),
                "channels": LaunchConfiguration("channels", default=1),
                "rate": LaunchConfiguration("rate", default=16000),
                "chunk": LaunchConfiguration("chunk", default=4096),
            }],
            remappings=[("audio", "/audio/in")]
        ),
        Node(
            package="chatgpt_ros",
            executable="chatgpt_ros",
            name="chatgpt_ros",
            remappings=[("/input_text", "/whisper/text"), ("/input_image", "/whisper/image")]
        ),
        # Node(
        #     package="crazyflie",
        #     executable="cfs_interpreter_node.py",
        #     name="CrazyflieAPI",
        #     remappings=[("/cmd_code_txt", "/output_text")]
        # ),
        # Node(
        #     package="crazyflie",
        #     executable="vel_mux.py",
        #     name="vel_mux",
        #     # remappings=[("/cmd_code_txt", "/whisper/output_text")]
        # ),
    ])
