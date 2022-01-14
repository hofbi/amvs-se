"""
ffmpeg wrapper for executing experiment commands
"""


# pylint: disable=C0103
# Variable names should be similar as in the paper

from subprocess import check_call, check_output
from tqdm import tqdm
from pathlib import Path
import json
import sys
import cv2
import re
from typing import List, Dict


try:
    sys.path.append(str(Path(__file__).absolute().parent))
except IndexError:
    pass

from model import EncodingParameterSet, EncodingStats, Settings
from video import VideoCaptureYUV


def call(command) -> int:
    """Call a subprocess in the shell and return code"""
    return check_call(command, shell=True)


def call_output(command) -> str:
    """Call a subprocess in the shell and return output"""
    return check_output(command, shell=True).decode(sys.stdout.encoding).strip()


def get_filter_extension(params):
    """Get filename for filtered video with p as replacement for dots"""
    sigma_as_string = str(params.sigma).replace(".", "p")
    return f"{params.k_size}_{sigma_as_string}"


def preprocess(
    params: EncodingParameterSet, settings: Settings, video: Path, tmp_dir: Path
):
    """Apply preprocessing on the raw video and store preprocessed raw video"""
    output_path = tmp_dir / f"{params.name}_{get_filter_extension(params)}.yuv"
    cap = VideoCaptureYUV(video, settings, output_path)

    while True:
        ret, frame = cap.read()
        if ret:
            filtered_frame = cv2.GaussianBlur(
                frame, (params.k_size, params.k_size), params.sigma
            )
            cap.write(filtered_frame)
        else:
            break


def preprocess_videos(
    params_list: List[EncodingParameterSet],
    settings: Settings,
    video: Path,
    tmp_dir: Path,
) -> None:
    """Run preprocessing for all preprocessing parameter combinations"""
    tmp_dir.mkdir(parents=True, exist_ok=True)
    for params in tqdm(params_list, desc="Preprocessing..."):
        preprocess(params, settings, video, tmp_dir)


def create_output_video_path(settings: Settings, video_config: Dict) -> str:
    """Create the output video path"""
    return f"{video_config['name']}{settings.extension}.mp4"


def encode(
    params: EncodingParameterSet, settings: Settings, video_config: Dict, tmp_dir: Path
) -> EncodingStats:
    """Encode video with given parameter and return encoding results"""
    extension = get_filter_extension(params)
    out_video = create_output_video_path(settings, video_config)
    call(
        (
            "ffmpeg"
            f" -s:v {settings.width}x{settings.height} -r {params.rate} -pix_fmt yuv420p"
            f" -i {tmp_dir}/{video_config['name']}_{extension}.yuv"
            f" -vf scale={params.resolution}"
            f" -vcodec {settings.ffmpeg_config(params)}"
            f" -y -hide_banner -loglevel panic {out_video}"
        )
    )
    stats = call_output(f"ffprobe -v error -print_format json -show_format {out_video}")
    format_stats = json.loads(stats)["format"]
    return EncodingStats(
        format_stats["bit_rate"],
        format_stats["duration"],
        format_stats["size"],
        video_config["sa"],
        video_config["ta"],
    )


def parse_metric_score(content: str, metric: str) -> float:
    """Parse the metric score from console output"""
    metric_patterns = {"ssim": r"All:(\d*\.\d*)", "psnr": r"average:(\d*\.\d*)"}
    score = re.search(metric_patterns[metric], content, re.DOTALL)
    return float(score.group(1)) if score else 0.0


def convert_mp4_to_yuv(mp4_path: str) -> str:
    """Convert an mp4 video to and yuv video using ffmpeg"""
    out_path = mp4_path + ".yuv"
    call(f"ffmpeg -y -i {mp4_path} {out_path}")
    return out_path


def scale_yuv(yuv_path: Path, source_res: str, target_res: str) -> str:
    """Scale a video from a source to a target resolution"""
    out_path = f"{yuv_path.stem}_scaled.yuv"
    call(
        f"ffmpeg -y -s:v {source_res} -pix_fmt yuv420p -i {yuv_path} -vf scale={target_res} {out_path}"
    )
    return out_path


def parse_resolution(resolution):
    """Parse width and height from a frame resolution string"""
    resolution = resolution.split("x")
    return int(resolution[0]), int(resolution[1])


def calculate_quality_metrics(
    params: EncodingParameterSet,
    settings: Settings,
    video: Path,
    tmp_dir: Path,
) -> Dict[str, float]:
    """Calculate the given video quality metric of a video for given encoding parameter"""
    video_config = create_video_config(video, settings)
    tmp_dir.mkdir(parents=True, exist_ok=True)
    preprocess(params, settings, video_config["path"], tmp_dir)
    stats = encode(params, settings, video_config, tmp_dir)
    output_video_path = create_output_video_path(settings, video_config)
    output = call_output(
        "ffmpeg -hide_banner"
        f" -s:v {settings.width}x{settings.height} -r {params.rate} -pix_fmt yuv420p"
        f" -i {video_config['path']} -i {output_video_path}"
        f' -lavfi "[0:v]scale={params.resolution}[main];[main][1:v]ssim;'
        f'[0:v]scale={params.resolution}[main];[main][1:v]psnr" -f null - 2>&1'
    )
    distorted_yuv = convert_mp4_to_yuv(output_video_path)
    scaled_yuv = scale_yuv(
        video_config["path"], f"{settings.width}x{settings.height}", params.resolution
    )
    width, height = parse_resolution(params.resolution)
    # Requires the VMAF docker image: https://github.com/Netflix/vmaf/pull/884
    vmaf_json = call_output(
        f"docker run --rm -v $(pwd):/files vmaf:latest yuv420p {width} {height}"
        f" /files/{scaled_yuv} /files/{distorted_yuv} --out-fmt json"
    )
    return {
        "bitrate": int(stats.bitrate) * 0.001,  # Convert to kbit/s
        "psnr": parse_metric_score(output, "psnr"),
        "ssim": parse_metric_score(output, "ssim"),
        "vmaf": json.loads(vmaf_json)["aggregate"]["VMAF_score"],
    }


def create_video_config(video: Path, settings: Settings) -> Dict:
    """Create video config with video specific parameter"""
    siti_output = call_output(
        f"siti {video} --width {settings.width} --height {settings.height} -of json"
    )
    siti_dict = json.loads(siti_output)
    video_name = video.stem.replace(settings.extension, "")
    return {
        "name": video_name,
        "sa": siti_dict["avg_si"],
        "ta": siti_dict["avg_ti"],
        "path": video,
    }
