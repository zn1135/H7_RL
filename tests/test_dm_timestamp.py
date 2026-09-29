"""原始 CAN 报文和解析时间戳的主机替身回归测试。"""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class DmTimestampTest(unittest.TestCase):
    def test_timestamp_stays_with_decoded_feedback(self):
        compiler = os.environ.get("CC") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no native C compiler")
        project = next((ROOT / "MDK-ARM").glob("*.uvprojx"))
        config = ET.parse(project).getroot().find(".//Cads/VariousControls")
        includes = [str((project.parent / path).resolve())
                    for path in config.find("IncludePath").text.split(";")]
        defines = ["-D" + item for item in config.find("Define").text.split(",")]
        port = (ROOT / "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/portmacro.h").read_text()
        start = port.index("static portFORCE_INLINE void vPortSetBASEPRI")
        end = port.index("#ifdef __cplusplus", start)
        adapter = (port[:start] + "\nvoid vPortSetBASEPRI(uint32_t);\n"
                   + "void vPortRaiseBASEPRI(void);\n"
                   + "uint32_t ulPortRaiseBASEPRI(void);\n"
                   + "BaseType_t xPortIsInsideInterrupt(void);\n" + port[end:])
        with tempfile.TemporaryDirectory(prefix="dm-timestamp-") as temp:
            folder = Path(temp)
            (folder / "portmacro.h").write_text(adapter)
            executable = folder / "dm-timestamp"
            command = [compiler, "-std=c99", "-O1", "-ffunction-sections",
                       "-fdata-sections", "-Wno-pointer-to-int-cast",
                       "-Wno-int-to-pointer-cast", *defines,
                       "-I" + str(folder), *("-I" + path for path in includes),
                       "tests/dm_timestamp_host_test.c", "-Wl,--gc-sections",
                       "-lm", "-o", str(executable)]
            subprocess.run(command, cwd=ROOT, check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
