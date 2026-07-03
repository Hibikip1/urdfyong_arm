import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "deploy_rdk_x5_humble.sh"
GUIDE = REPO_ROOT / "docs" / "rdk-x5-deployment.md"
README = REPO_ROOT / "README.md"


class DeployRdkX5ScriptTest(unittest.TestCase):
    def test_script_exists_and_has_bash_shebang(self):
        self.assertTrue(SCRIPT.exists(), "RDK X5 deploy script is missing")
        first_line = SCRIPT.read_text(encoding="utf-8").splitlines()[0]
        self.assertEqual(first_line, "#!/usr/bin/env bash")

    def test_script_documents_target_platform_and_ros_distro(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("RDK X5", text)
        self.assertIn("Ubuntu 22.04", text)
        self.assertIn("ROS_DISTRO=humble", text)

    def test_script_installs_ros_dependencies_and_builds_workspace(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("rosdep install", text)
        self.assertIn("colcon build", text)
        self.assertIn("--parallel-workers", text)
        self.assertIn("--skip-apt", text)

    def test_script_mentions_usb2can_serial_permissions(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("/dev/ttyACM0", text)
        self.assertIn("dialout", text)

    def test_readme_links_rdk_x5_deployment_guide(self):
        readme = README.read_text(encoding="utf-8")
        self.assertIn("docs/rdk-x5-deployment.md", readme)

    def test_deployment_guide_describes_remote_control_split(self):
        guide = GUIDE.read_text(encoding="utf-8")
        self.assertIn("RDK X5", guide)
        self.assertIn("地瓜派", guide)
        self.assertIn("电脑", guide)
        self.assertIn("ros2_control", guide)


if __name__ == "__main__":
    unittest.main()
