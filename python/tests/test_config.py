"""Python 侧任务配置：渲染、校验、插件节点还原、受限子集解析、哈希稳定性。

不依赖 C++ 工具：schema 在这里手写一份小的。与 C++ 的逐字节比对见 test_cpp_parity.py。
"""

import unittest

from simpleslam.config import Config, ConfigError, _scalar
from simpleslam.schema import Field, PluginField, PluginSchema, Schema

LIO = "system.modules.odometry.plugin"
SOURCE = "data.source"


def fake_schema() -> Schema:
    return Schema(
        [
            Field("task.name", "string", "default", "任务名", False),
            Field("task.seed", "int", "0", "随机种子", False),
            Field("system.runtime.spin_budget", "int", "4096", "步进预算", False),
            Field(LIO, "slot", "", "里程计", False),
            Field("system.modules.graph_optimizer.weight", "double", "100", "权重", True),
            Field(SOURCE, "slot", "", "数据来源", False),
            Field("data.sequences", "list", "[]", "多序列", False),
            Field("outputs.dir", "string", "", "任务记录目录", False),
        ],
        slots={LIO: "OdometryBase", SOURCE: "DatasetReader"},
        plugins=[
            PluginSchema("OdometryBase", "lio", (
                PluginField("keyframe_distance_m", "double", "1", "关键帧距离"),
                PluginField("max_iterations", "int", "30", "迭代上限"),
                PluginField("pressure", "float", "0.5", "单精度参数"),
                # 默认子节点带非默认参数：只凭类型名补不对
                PluginField("map", "slot:Map", "voxel", "地图",
                            {"type": "voxel", "voxel_size_m": 0.5}),
            )),
            PluginSchema("OdometryBase", "needs_map", (
                PluginField("map", "slot:Map", "", "地图，必须选"),
            )),
            PluginSchema("Map", "voxel", (PluginField("voxel_size_m", "double", "1", "体素"),)),
            PluginSchema("Map", "sparse", ()),
            PluginSchema("DatasetReader", "kitti", (
                PluginField("path", "string", "", "路径"),
                PluginField("sequence", "string", "", "序列名"),
            )),
        ],
    )


def minimal(**extra) -> Config:
    return Config({LIO: {"type": "lio"}, SOURCE: {"type": "kitti", "path": "/data"}}, **extra)


class SchemaTest(unittest.TestCase):
    def test_default_spec_from_cpp_json_becomes_a_plugin_node(self):
        # 与 C++ 侧 test_config_schema 里钉住的是同一段 JSON：两边各从一头钉住格式
        spec = ('{"type": "voxel", "params": {"label": "a\\"b", "n": 3, "on": true, '
                '"size": 0.5}, "children": {"inner": {"type": "leaf", "params": {}, '
                '"children": {}}}}')
        text = (
            '{"fields": [{"path": "task.name", "type": "string", "default": "x", "doc": "", '
            '"tunable": false}], "slots": {}, "plugins": [{"interface": "OdometryBase", '
            '"type": "nested", "fields": [{"name": "map", "kind": "slot:Map", '
            f'"default": "voxel", "doc": "", "default_spec": {spec}}}]}}]}}'
        )
        field = Schema.from_json(text).plugin("OdometryBase", "nested").fields[0]
        self.assertEqual(field.slot_interface, "Map")
        self.assertEqual(field.default_value(), {"type": "voxel", "label": 'a"b', "n": 3,
                                                 "on": True, "size": 0.5,
                                                 "inner": {"type": "leaf"}})
        self.assertIsNot(field.default_value(), field.default_spec)  # 拿到的是副本


class RenderTest(unittest.TestCase):
    def test_only_explicit_items_are_written(self):
        config = Config(task={"name": "demo"})
        config[LIO] = {"type": "lio", "max_iterations": 40, "keyframe_distance_m": 1.5}
        self.assertEqual(
            config.to_yaml(fake_schema()),
            'task:\n  name: "demo"\n'
            "\nsystem:\n  modules:\n    odometry:\n      plugin:\n"
            '        type: "lio"\n        keyframe_distance_m: 1.5\n        max_iterations: 40\n',
        )

    def test_system_file_and_overrides_are_rendered_in_place(self):
        config = minimal(system="../systems/x.yaml")
        del config[LIO]
        config.override(f"{LIO}.keyframe_distance_m", 2.0)
        config.override(f"{SOURCE}.sequence", "00")  # 带引号：序列名是字符串，不是整数 0
        config.override(LIO, {"type": "lio", "map": {"type": "sparse"}})
        self.assertEqual(
            config.to_yaml(fake_schema()),
            'system: "../systems/x.yaml"\n'
            '\ndata:\n  source:\n    type: "kitti"\n    path: "/data"\n'
            '\noverrides: ["system.modules.odometry.plugin.keyframe_distance_m=2", '
            '"data.source.sequence=\\"00\\"", '
            '"system.modules.odometry.plugin={type: \\"lio\\", map: {type: \\"sparse\\"}}"]\n',
        )

    def test_update_by_section_and_by_path(self):
        config = Config()
        config.update({"task.seed": 3}, system={"runtime": {"spin_budget": 12}})
        config.update(system={"modules": {"odometry": {"plugin": {"type": "lio"}}}})
        self.assertEqual(config["task.seed"], 3)
        self.assertEqual(config["system.runtime.spin_budget"], 12)
        self.assertEqual(config[LIO], {"type": "lio"})  # 带 type 的映射整体是一个值

    def test_copy_is_deep(self):
        base = minimal()
        variant = base.copy()
        variant[LIO]["keyframe_distance_m"] = 9.0
        variant.override("task.seed", 1)
        self.assertNotIn("keyframe_distance_m", base[LIO])
        self.assertNotIn("overrides", base)


class ValidateTest(unittest.TestCase):
    def test_unknown_key_is_an_error(self):
        with self.assertRaisesRegex(ConfigError, "未知配置项 task.nam"):
            Config(task={"nam": "x"}).validate(fake_schema())

    def test_key_inside_a_plugin_node_says_so(self):
        with self.assertRaisesRegex(ConfigError, "在插件节点 data.source 里面"):
            Config({"data.source.path": "/x"}).validate(fake_schema())

    def test_type_is_not_coerced(self):
        schema = fake_schema()
        for bad in ({"task.seed": "3"}, {"task.seed": 1.5}, {"task.seed": True},
                    {"task.name": 3}, {"data.sequences": "00"}, {SOURCE: "kitti"},
                    {"system.modules.graph_optimizer.weight": float("inf")}):
            with self.assertRaises(ConfigError, msg=repr(bad)):
                Config(bad).validate(schema)
        # 整数喂给实数字段可以：那是拓宽，不是改变含义
        Config({"system.modules.graph_optimizer.weight": 1}).validate(schema)

    def test_system_file_excludes_inline_system_values(self):
        config = minimal(system="../systems/x.yaml")
        with self.assertRaisesRegex(ConfigError, "覆盖项"):
            config.validate(fake_schema())

    def test_override_paths_are_checked(self):
        with self.assertRaisesRegex(ConfigError, "未知配置项 task.nam"):
            Config().override("task.nam", "x").validate(fake_schema())
        Config().override(f"{LIO}.anything", 1).validate(fake_schema())  # 插件内部归目录查


class ResolveTest(unittest.TestCase):
    def test_canonical_fills_defaults_and_resolves_plugins(self):
        text = minimal(task={"name": "demo"}).canonical(fake_schema())
        self.assertEqual(
            text,
            'task:\n  name: "demo"\n  seed: 0\n'
            "\nsystem:\n  runtime:\n    spin_budget: 4096\n  modules:\n    odometry:\n"
            '      plugin:\n        type: "lio"\n        keyframe_distance_m: 1\n'
            "        max_iterations: 30\n        pressure: 0.5\n        map:\n"
            '          type: "voxel"\n          voxel_size_m: 0.5\n'
            "    graph_optimizer:\n      weight: 100\n"
            '\ndata:\n  source:\n    type: "kitti"\n    path: "/data"\n    sequence: ""\n'
            "  sequences: []\n"
            '\noutputs:\n  dir: ""\n',
        )

    def test_float_parameters_are_rounded_like_cpp(self):
        config = minimal()
        config[LIO]["pressure"] = 0.1
        resolved = config.resolved(fake_schema())[LIO]
        self.assertEqual(resolved["pressure"], 0.10000000149011612)

    def test_child_slot_is_resolved_against_its_own_schema(self):
        config = minimal()
        config[LIO]["map"] = {"type": "voxel"}  # 写了就按实现的默认值，不是父节点给的默认节点
        self.assertEqual(config.resolved(fake_schema())[LIO]["map"],
                         {"type": "voxel", "voxel_size_m": 1.0})

    def test_plugin_errors_name_the_path_and_the_choices(self):
        schema = fake_schema()
        cases = [
            ({"type": ""}, "没有写 type.*lio、needs_map"),
            ({"type": "kitti"}, "kitti 是 DatasetReader 的实现"),
            ({"type": "lio", "max_iteration": 1}, "max_iteration：lio 没有这个参数，可用的有"),
            ({"type": "lio", "max_iterations": 1.5}, "应为 int，实际是 double"),
            ({"type": "lio", "map": "voxel"}, "这是一个槽"),
            ({"type": "lio", "max_iterations": {"type": "voxel"}}, "这是一个参数，不是槽"),
            ({"type": "needs_map"}, "map：缺少这个槽"),
            ({"type": "lio", "map": {"type": "sparse", "x": 1}}, "它不接受任何参数"),
        ]
        for node, pattern in cases:
            config = minimal()
            config[LIO] = node
            with self.assertRaisesRegex(ConfigError, pattern, msg=repr(node)):
                config.resolved(schema)

    def test_file_references_are_left_to_cpp(self):
        config = minimal()
        config.override("task.seed", 1)
        with self.assertRaisesRegex(ConfigError, "through_cpp"):
            config.hash(fake_schema())


class HashTest(unittest.TestCase):
    def test_hash_depends_only_on_effective_content(self):
        schema = fake_schema()
        a = minimal(task={"name": "demo"})
        b = Config({SOURCE: {"path": "/data", "type": "kitti"}, "task.name": "demo",
                    LIO: {"type": "lio"}})
        self.assertEqual(a.hash(schema), b.hash(schema))
        # 显式写成默认值也不改变哈希：哈希看的是还原后的生效值
        c = minimal(task={"name": "demo", "seed": 0})
        c[LIO].update(max_iterations=30, map={"type": "voxel", "voxel_size_m": 0.5})
        self.assertEqual(a.hash(schema), c.hash(schema))
        d = minimal(task={"name": "demo"})
        d[LIO]["max_iterations"] = 31
        self.assertNotEqual(a.hash(schema), d.hash(schema))

    def test_outputs_do_not_change_the_task_hash(self):
        schema = fake_schema()
        a, b = minimal(), minimal(outputs={"dir": "/elsewhere"})
        self.assertEqual(a.hash(schema), b.hash(schema))
        self.assertNotEqual(a.canonical(schema), b.canonical(schema))

    def test_system_hash_covers_only_the_system(self):
        schema = fake_schema()
        a, b = minimal(), minimal(task={"name": "other"})
        b[SOURCE]["path"] = "/other"
        self.assertEqual(a.system_hash(schema), b.system_hash(schema))
        self.assertNotEqual(a.hash(schema), b.hash(schema))
        b[LIO]["max_iterations"] = 7
        self.assertNotEqual(a.system_hash(schema), b.system_hash(schema))


class ParseTest(unittest.TestCase):
    def test_canonical_roundtrip_is_lossless_for_nasty_strings(self):
        schema = fake_schema()
        for nasty in ('a # b', 'x: y', '"quoted"', 'tail  ', 'back\\slash', 'line\nbreak',
                      'true', '42', '[1, 2]', '#lead'):
            config = minimal(task={"name": nasty}, data={"sequences": [nasty, "b"]})
            config[SOURCE]["path"] = nasty
            reloaded = Config.from_yaml(config.canonical(schema), schema)
            self.assertEqual(reloaded["task.name"], nasty, f"往返丢失: {nasty!r}")
            self.assertEqual(reloaded[SOURCE]["path"], nasty, f"插件参数往返丢失: {nasty!r}")
            self.assertEqual(reloaded["data.sequences"], [nasty, "b"])
            self.assertEqual(reloaded.hash(schema), config.hash(schema))

    def test_rendered_task_roundtrip(self):
        schema = fake_schema()
        original = minimal(task={"name": "demo", "seed": 7}, data={"sequences": ["00", "01"]})
        original.override(f"{LIO}.max_iterations", 5)
        original["system"] = "../systems/x.yaml"
        del original[LIO]
        self.assertEqual(Config.from_yaml(original.to_yaml(schema), schema), original)

    def test_hand_written_subset(self):
        text = (
            "# 注释\n"
            "task:\n  name: demo#1   # 引号外、空白之后的 # 才是注释\n  seed: 7\n"
            "system:\n  modules:\n    odometry:\n      plugin:\n        type: lio\n"
            "        max_iterations: 12\n"
            "data:\n  source:\n    type: kitti\n    sequence: \"00\"\n"
        )
        config = Config.from_yaml(text, fake_schema())
        self.assertEqual(config["task.name"], "demo#1")
        self.assertEqual(config[LIO], {"type": "lio", "max_iterations": 12})
        self.assertEqual(config[SOURCE]["sequence"], "00")

    def test_subset_rejects_what_it_cannot_read(self):
        schema = fake_schema()
        for text in ("task: {name: x}\n", "data:\n  sequences:\n    - a\n",
                     "task:\n  name: 'x'\n", "task:\n   name: x\n", "task:\n  seed: \"3\"\n",
                     "task:\n  name: a\n  name: b\n"):
            with self.assertRaises(ConfigError, msg=text):
                Config.from_yaml(text, schema)

    def test_scalar_follows_cpp(self):
        # 与 C++ 的 scalarValue 同一口径：from_chars 完整读成整数或实数才算数
        cases = {"00": 0, "+5": 5, "-7": -7, "1e3": 1000.0, ".5": 0.5, "1.": 1.0,
                 "1_000": "1_000", "0x10": "0x10", "true": True, '"true"': "true",
                 "9223372036854775808": 9223372036854775808.0, "++5": "++5", "-": "-",
                 "1e400": "1e400", "1e-400": "1e-400", "0e5": 0.0, "-0.0": -0.0}
        for text, expected in cases.items():
            value = _scalar(text)
            self.assertEqual(value, expected, text)
            self.assertIs(type(value), type(expected), text)


if __name__ == "__main__":
    unittest.main()
