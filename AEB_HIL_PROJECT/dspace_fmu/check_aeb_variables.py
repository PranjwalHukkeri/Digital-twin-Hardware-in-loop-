from win32com.client import Dispatch

app = Dispatch("ControlDeskNG.Application.2026-A")

experiment = app.ActiveExperiment

if experiment is None:
    print("ERROR: No active experiment. Open VIRYA_AEB_Test first.")
    raise SystemExit

print("Experiment:", experiment.Name)

for platform in experiment.Platforms:
    print("\nPlatform:", platform.Name)

    variables = platform.ActiveVariableDescription.Variables

    names = [
        "ego_speed",
        "target_speed",
        "distance",
        "relative_velocity",
        "TTC",
        "AEB_state",
        "desired_deceleration"
    ]

    for name in names:
        try:
            v = variables.Item(name)
            print(
                name,
                "| Type:", v.Type,
                "| Readable:", v.IsReadable,
                "| Writable:", v.IsWritable,
                "| Value:", v.ValueConverted
            )
        except Exception as e:
            print(name, "| NOT FOUND:", str(e))

print("\nDiagnostic finished. No values were changed.")
input("Press Enter to close...")