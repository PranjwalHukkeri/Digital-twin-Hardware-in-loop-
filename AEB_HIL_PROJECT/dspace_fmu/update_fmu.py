
import os
import sys
import zipfile
import tempfile
import shutil
from datetime import datetime
import xml.etree.ElementTree as ET

def main():
    if len(sys.argv) < 3:
        print("Usage: python update_fmu.py <fmu> <VIRYA_AEB_main.c>")
        sys.exit(1)

    fmu_path = os.path.abspath(sys.argv[1])
    source_path = os.path.abspath(sys.argv[2])

    if not os.path.isfile(fmu_path):
        sys.exit("ERROR: FMU file not found: " + fmu_path)
    if not os.path.isfile(source_path):
        sys.exit("ERROR: C source file not found: " + source_path)

    with zipfile.ZipFile(fmu_path, "r") as z:
        names = z.namelist()
        if "modelDescription.xml" not in names:
            sys.exit("ERROR: modelDescription.xml missing from FMU")
        source_member = "sources/VIRYA_AEB_main.c"
        if source_member not in names:
            sys.exit("ERROR: " + source_member + " missing from FMU")
        xml_bytes = z.read("modelDescription.xml")
        old_source = z.read(source_member)

    root = ET.fromstring(xml_bytes)
    xml_guid = root.attrib.get("guid")
    source_text = open(source_path, "r", encoding="utf-8-sig").read()
    marker = '#define FMU_GUID "'
    pos = source_text.find(marker)
    if pos < 0:
        sys.exit("ERROR: FMU_GUID definition not found in C source")
    guid_start = pos + len(marker)
    guid_end = source_text.find('"', guid_start)
    source_guid = source_text[guid_start:guid_end]

    if not xml_guid or source_guid != xml_guid:
        sys.exit(
            "ERROR: GUID mismatch.\n"
            f"XML GUID:    {xml_guid}\n"
            f"Source GUID: {source_guid}"
        )

    backup_path = fmu_path + "." + datetime.now().strftime("%Y%m%d_%H%M%S") + ".bak"
    shutil.copy2(fmu_path, backup_path)
    print("Backup created:", backup_path)

    fd, temp_path = tempfile.mkstemp(
        prefix="updated_", suffix=".fmu", dir=os.path.dirname(fmu_path)
    )
    os.close(fd)

    try:
        with zipfile.ZipFile(fmu_path, "r") as zin, \
             zipfile.ZipFile(temp_path, "w", zipfile.ZIP_DEFLATED) as zout:
            for item in zin.infolist():
                data = zin.read(item.filename)
                if item.filename == source_member:
                    data = source_text.encode("utf-8")
                zout.writestr(item, data)

        with zipfile.ZipFile(temp_path, "r") as check:
            bad_member = check.testzip()
            if bad_member:
                raise RuntimeError("ZIP verification failed: " + bad_member)
            if check.read(source_member).decode("utf-8") != source_text:
                raise RuntimeError("Embedded C source verification failed")
            ET.fromstring(check.read("modelDescription.xml"))

        os.replace(temp_path, fmu_path)
        print("SUCCESS: Updated embedded C source.")
        print("FMU:", fmu_path)
        print("Original embedded source bytes:", len(old_source))
        print("New embedded source bytes:", len(source_text.encode("utf-8")))
        print("NOTE: This updates the source only; it does not compile a new FMU binary.")
    except Exception:
        if os.path.exists(temp_path):
            os.remove(temp_path)
        raise

if __name__ == "__main__":
    main()
