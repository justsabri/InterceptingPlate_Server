import shutil
import tempfile
import zipfile
from pathlib import Path
from xml.sax.saxutils import escape

DOCX_PATH = Path("截流板与信息系统通讯协议.docx")
BACKUP_PATH = Path("截流板与信息系统通讯协议.docx.bak")

if not DOCX_PATH.exists():
    raise FileNotFoundError(DOCX_PATH)

shutil.copy2(DOCX_PATH, BACKUP_PATH)

note_text = (
    "JLB-4协议确认补充：2.3消息方向按“15M艇信息化系统发送给截流板状态信息”执行，"
    "即信息化系统为发送方、截流板为接收方；消息长度不包含前4字节消息头；"
    "消息1固定长度发送，电机数量为2时电机3/4状态发送初始值101；异常报文只记录并丢弃；"
    "15m版本移除旧TCP shutdown字段；控制模式仅支持1自动、2手动，不兼容0；"
    "自动模式参数直接透传算法模块；状态发送频率通过config.json配置；消息2时间戳保留double。"
)

note_xml = (
    "<w:p>"
    "<w:r><w:t>"
    + escape(note_text)
    + "</w:t></w:r>"
    "</w:p>"
)

with zipfile.ZipFile(BACKUP_PATH, "r") as zin:
    xml = zin.read("word/document.xml").decode("utf-8")

    if "<w:t>28-36</w:t>" not in xml:
        raise RuntimeError("未找到需要修正的纬度字节位置 28-36")

    xml = xml.replace("<w:t>28-36</w:t>", "<w:t>29-36</w:t>", 1)

    if note_text not in xml:
        xml = xml.replace("</w:body>", note_xml + "</w:body>")

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".docx")
    tmp.close()
    tmp_path = Path(tmp.name)

    with zipfile.ZipFile(tmp_path, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = xml.encode("utf-8") if item.filename == "word/document.xml" else zin.read(item.filename)
            zout.writestr(item, data)

shutil.move(str(tmp_path), DOCX_PATH)
print(f"updated: {DOCX_PATH}")
print(f"backup: {BACKUP_PATH}")