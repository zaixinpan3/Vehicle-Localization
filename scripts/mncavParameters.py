"""Canonical MnCAV replay configuration shared by Python experiment scripts."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


def replay_parameters(calibration_file=None):
    """Historical files may override IMU corrections, never vehicle/steering."""
    parameters = json.loads((ROOT / 'config/mncavVehicleParameters.json').read_text())
    parameters['input_correction'] = json.loads(
        (ROOT / 'config/mncavInputCorrections.json').read_text())['input_correction']
    parameters['inputCorrectionSource'] = str(ROOT / 'config/mncavInputCorrections.json')
    if calibration_file is not None:
        archived = json.loads(Path(calibration_file).read_text())
        if 'input_correction' in archived:
            parameters['input_correction'] = archived['input_correction']
            parameters['inputCorrectionSource'] = str(calibration_file)
    interface = json.loads((ROOT / 'config/mncavReplayInterface.json').read_text())
    parameters['steeringWheelOffsetRad'] = interface['steeringWheelOffsetRad']
    parameters['steeringInterfaceCalibration'] = interface
    parameters['vehicleParameterSource'] = str(ROOT / 'config/mncavVehicleParameters.json')
    return parameters
