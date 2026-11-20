import os
import sys

XIL_BIN = r"D:\Dspace Release 2026A\dSPACE XIL API .NET 2026-A\MAPort\Main\bin\AutomationAssemblies"
sys.path.append(XIL_BIN)

import dspace.clr
dspace.clr.LoadRuntime(base_version="8.0.0")

import clr

clr.AddReference("ASAM.XIL.Implementation.TestbenchFactory, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")
clr.AddReference("ASAM.XIL.Interfaces, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")

from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
from ASAM.XIL.Interfaces.Testbench.MAPort.Enum import MAPortState

CONFIG = r"D:\VIRYA_AEB_HIL\XIL_Test\VIRYA_AEB_MAPort.xml"

maPort = None

try:
    print("Creating XIL testbench...")
    factory = TestbenchFactory()
    testbench = factory.CreateVendorSpecificTestbench(
        "dSPACE GmbH", "XIL API", "2026-A"
    )

    print("Creating MAPort...")
    maPort = testbench.MAPortFactory.CreateMAPort("VIRYA_AEB_ReadOnly")

    print("Loading VIRYA_AEB configuration...")
    config = maPort.LoadConfiguration(CONFIG)

    print("Configuring MAPort without downloading an application...")
    maPort.Configure(config, False)

    print("MAPort state:", maPort.State)
    print("MAPort configuration completed.")

except Exception as ex:
    print("MAPort connection/configuration failed:")
    print(type(ex).__name__, ex)

finally:
    if maPort is not None:
        maPort.Dispose()
        print("MAPort resources released.")