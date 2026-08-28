from linuxpy.video.device import iter_video_capture_devices


def format_control_info(control):
    """格式化控制参数信息"""
    return {
        "name": (
            control.name.decode() if hasattr(control, "name") else str(control.name)
        ),
        "id": control.id,
        "type": control.type,
        "min": control.minimum,
        "max": control.maximum,
        "step": control.step,
        "default": control.default_value,
        "flags": control.flags,
    }


def format_frame_info(frame):
    """格式化帧信息"""
    return {
        "format": str(frame.pixel_format).split(".")[-1],
        "resolution": f"{frame.width}x{frame.height}",
        "fps": f"{frame.min_fps}-{frame.max_fps}",
    }


def format_format_info(fmt):
    """格式化像素格式信息"""
    return {
        "type": str(fmt.pixel_format).split(".")[-1],
        "description": fmt.description,
        "compressed": bool(fmt.flags),
    }


def print_camera_info():
    """打印摄像头信息，格式化输出"""

    print("=" * 80)
    print("检测到的摄像头设备")
    print("=" * 80)

    device_count = 0

    for dev in iter_video_capture_devices():
        with dev:
            info = dev.info
            device_count += 1

            print(f"\n📷 设备 {device_count}: {info.card}")
            print("-" * 60)

            # 基本信息
            print("🔧 基本信息:")
            print(f"  设备名称: {info.card}")
            print(f"  驱动程序: {info.driver}")
            print(f"  设备路径: {info.device.filename}")
            print(f"  总线信息: {info.bus_info}")
            print(f"  内核版本: {info.version}")

            # 支持的像素格式
            print("\n🎨 支持的像素格式:")
            for fmt in info.formats:
                fmt_info = format_format_info(fmt)
                compressed_str = " (压缩)" if fmt_info["compressed"] else ""
                print(
                    f"  • {fmt_info['type']}: {fmt_info['description']}{compressed_str}"
                )

            # 支持的分辨率和帧率
            print("\n📐 支持的分辨率和帧率:")
            # 按像素格式分组
            formats_grouped = {}
            for frame in info.frame_sizes:
                fmt_name = str(frame.pixel_format).split(".")[-1]
                if fmt_name not in formats_grouped:
                    formats_grouped[fmt_name] = []
                formats_grouped[fmt_name].append(format_frame_info(frame))

            for fmt_name, frames in formats_grouped.items():
                print(f"  {fmt_name}:")
                for frame_info in frames:
                    print(f"    • {frame_info['resolution']} @ {frame_info['fps']} fps")

            # 控制参数
            print("\n⚙️  可调节参数:")
            if info.controls:
                for control in info.controls:
                    ctrl_info = format_control_info(control)
                    print(f"  • {ctrl_info['name']}:")
                    print(
                        f"    范围: {ctrl_info['min']} - {ctrl_info['max']} (步长: {ctrl_info['step']})"
                    )
                    print(f"    默认值: {ctrl_info['default']}")
            else:
                print("  无可调节参数")

            # 输入源
            print("\n📥 输入源:")
            if info.inputs:
                for inp in info.inputs:
                    print(f"  • {inp.name} (索引: {inp.index})")
            else:
                print("  无输入源信息")

            print("\n" + "=" * 80)

    if device_count == 0:
        print("未检测到任何摄像头设备")
    else:
        print(f"\n总共检测到 {device_count} 个摄像头设备")


if __name__ == "__main__":
    print_camera_info()
