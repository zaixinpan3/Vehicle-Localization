"""Protect structural geometry when preparing clean CARLA mapping scenes."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch, Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/carla"))
from carlaSceneProfile import removal_reason, apply_scene_profile, restore_scene_profile, select_environment_objects


class SceneSelectionTest(unittest.TestCase):
    def test_parked_traffic_and_movable_clutter(self):
        for label, name in [("Car", "StaticMeshActor_127_SM_0"),
                            ("Truck", "BP_MercedesSprinter_Parked_C_3_SM_0"),
                            ("Dynamic", "StaticMeshActor_105_SM_0"),
                            ("Static", "SM_MailBox2_571_SM_0"),
                            ("Other", "SM_TrafficCone_12_SM_0")]:
            with self.subTest(label=label, name=name):
                self.assertIsNotNone(removal_reason(label, name))

    def test_structural_classes_and_mislabeled_lamp_components(self):
        for label, name in [("Sidewalks", "Road_Curb_Town10HD10_218_SM_0"),
                            ("Buildings", "Actor_0_Inst_0_0"),
                            ("Poles", "BP_AddPole1_10_SM_0"),
                            ("Other", "BP_StreetLight_simple10_22_SM_2"),
                            ("Dynamic", "BP_HighwayLight_multiple4_2_SM_0"),
                            ("Vegetation", "StaticMeshActor_104_SM_0"),
                            ("Static", "UnknownMesh_12")]:
            with self.subTest(label=label, name=name):
                self.assertIsNone(removal_reason(label, name))

    def test_original_profile_has_no_world_access(self):
        self.assertFalse(apply_scene_profile(None, "original")["mutationApplied"])

    def test_invalid_profile_fails_before_access(self):
        with self.assertRaises(ValueError):
            apply_scene_profile(None, "missing")

    def test_shared_actor_protection_and_whole_vehicle_removal(self):
        rows = [SimpleNamespace(id=1, name="Shared_SM_0", type="Buildings"),
                SimpleNamespace(id=2, name="Shared_SM_1", type="Dynamic"),
                SimpleNamespace(id=3, name="Parked_SM_0", type="Car"),
                SimpleNamespace(id=4, name="Parked_SM_1", type="NONE")]
        selected, kept = select_environment_objects(rows)
        self.assertEqual({row["id"] for row in selected}, {3, 4})
        self.assertEqual(kept["Buildings"], 1)
        self.assertEqual(kept["Dynamic"], 1)

    def test_live_traffic_rejected_before_environment_mutation(self):
        world = Mock()
        world.get_actors.return_value = [SimpleNamespace(id=7, type_id="vehicle.test")]
        with patch.dict(sys.modules, {"carla": SimpleNamespace()}):
            with self.assertRaisesRegex(RuntimeError, "live traffic"):
                apply_scene_profile(world)
        world.get_environment_objects.assert_not_called()
        world.enable_environment_objects.assert_not_called()

    def test_capture_restores_only_its_selected_environment_ids(self):
        world = Mock()
        world.get_actors.return_value = []
        world.get_environment_objects.return_value = [
            SimpleNamespace(id=3, name="Parked_SM_0", type="Car"),
            SimpleNamespace(id=4, name="Wall_SM_0", type="Buildings")]
        api = SimpleNamespace(CityObjectLabel=SimpleNamespace(Any=255))
        with patch.dict(sys.modules, {"carla": api}):
            audit = apply_scene_profile(world)
        world.enable_environment_objects.assert_called_once_with({3}, False)
        restore_scene_profile(world, audit)
        self.assertEqual(world.enable_environment_objects.call_args_list[-1].args, ({3}, True))


if __name__ == "__main__":
    unittest.main()
