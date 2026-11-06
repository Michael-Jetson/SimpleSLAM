"""Python 侧任务配置：渲染、校验、受限子集解析。还原插件节点、补默认值在 C++。

不依赖 C++ 工具：schema 在这里手写一份小的。经 C++ 加载、导出的对照见 test_cpp_entry.py。
"""

import unittest

from simpleslam.config import Config, ConfigError, _scalar
from simpleslam.schema import Field, PluginField, PluginSchema, Schema

LIO = "system.modules.odometry.plugin"
SOURCE = "data.source"


def fake_schema(extra_fields=(), legacy=None) -> Schema:
    return Schema(
        [
            Field("task.name", "string", "default", "任务名", False),
            Field("task.repeats", "int", "1", "测试用的整数字段", False),
            Field("system.runtime.spin_budget", "int", "4096", "步进预算", False),
            Field(LIO, "slot", "", "里程计", False),
            Field("system.modules.graph_optimizer.weight", "double", "100", "权重", True),
            Field(SOURCE, "slot", "", "数据来源", False),
            Field("data.sequences", "list", "[]", "多序列", False),
            Field("outputs.dir", "string", "", "任务记录目录", False),
            *extra_fields,
        ],
        slots={LIO: "OdometryBase", SOURCE: "DatasetReader"},
        legacy=legacy,
        plugins=[
            PluginSchema("OdometryBase", "lio", (
                PluginField("keyframe_distance_m", "double", "1", "关键帧距离"),
                PluginField("max_iterations", "int", "30", "迭代上限"),
                PluginField("pressure", "float", "0.5", "单精度参数"),
                PluginField("map", "slot:Map", "voxel", "地图"),
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
    def test_plugin_fields_and_legacy_table_are_read_from_cpp_json(self):
        # C++ 导出的子槽默认节点（default_spec）不读：还原插件节点在 C++
        text = (
            '{"fields": [{"path": "task.name", "type": "string", "default": "x", "doc": "", '
            '"tunable": false}], "slots": {}, "plugins": [{"interface": "OdometryBase", '
            '"type": "nested", "fields": [{"name": "map", "kind": "slot:Map", '
            '"default": "voxel", "doc": "地图", "default_spec": {"type": "voxel", "params": {}, '
            '"children": {}}}]}], "legacy": {"task.repeats": "去掉了"}}'
        )
        schema = Schema.from_json(text)
        field = schema.plugin("OdometryBase", "nested").fields[0]
        self.assertEqual((field.name, field.kind, field.default), ("map", "slot:Map", "voxel"))
        self.assertEqual(schema.implementations("OdometryBase"), ["nested"])
        self.assertEqual(schema.legacy, {"task.repeats": "去掉了"})


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
        config.update({"task.repeats": 3}, system={"runtime": {"spin_budget": 12}})
        config.update(system={"modules": {"odometry": {"plugin": {"type": "lio"}}}})
        self.assertEqual(config["task.repeats"], 3)
        self.assertEqual(config["system.runtime.spin_budget"], 12)
        self.assertEqual(config[LIO], {"type": "lio"})  # 带 type 的映射整体是一个值

    def test_copy_is_deep(self):
        base = minimal()
        variant = base.copy()
        variant[LIO]["keyframe_distance_m"] = 9.0
        variant.override("task.repeats", 1)
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
        for bad in ({"task.repeats": "3"}, {"task.repeats": 1.5}, {"task.repeats": True},
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


class LegacyTest(unittest.TestCase):
    """旧写法：Python 认得 schema 换算表里的旧键，原样渲染进所在的节，换算交给 C++。"""

    @staticmethod
    def schema() -> Schema:
        return fake_schema(
            extra_fields=[Field("evaluation.groundtruth", "string", "", "真值", False)],
            legacy={"task.purpose": "去掉了", "evaluation.alignment": "evaluators.ate.alignment"})

    def test_old_keys_are_known_and_rendered_in_their_section(self):
        schema = self.schema()
        config = minimal(task={"name": "old", "purpose": "experiment"},
                         evaluation={"groundtruth": "gt.tum", "alignment": "sim3"})
        config.validate(schema)
        self.assertEqual(config.legacy_paths(schema), ["evaluation.alignment", "task.purpose"])
        text = config.to_yaml(schema)
        self.assertIn('task:\n  name: "old"\n  purpose: "experiment"\n', text)
        self.assertIn('evaluation:\n  groundtruth: "gt.tum"\n  alignment: "sim3"\n', text)
        with self.assertRaisesRegex(ConfigError, "未知配置项 task.purpose"):
            config.validate(fake_schema())  # 没有换算表就不认得

    def test_old_override_paths_are_known(self):
        config = minimal()
        config.override("evaluation.alignment", "sim3")
        config.validate(self.schema())
        with self.assertRaisesRegex(ConfigError, "覆盖项里的未知配置项"):
            config.validate(fake_schema())


class ParseTest(unittest.TestCase):
    def test_render_roundtrip_is_lossless_for_nasty_strings(self):
        schema = fake_schema()
        for nasty in ('a # b', 'x: y', '"quoted"', 'tail  ', 'back\\slash', 'line\nbreak',
                      'true', '42', '[1, 2]', '#lead'):
            config = minimal(task={"name": nasty}, data={"sequences": [nasty, "b"]})
            config[SOURCE]["path"] = nasty
            reloaded = Config.from_yaml(config.to_yaml(schema), schema)
            self.assertEqual(reloaded["task.name"], nasty, f"往返丢失: {nasty!r}")
            self.assertEqual(reloaded[SOURCE]["path"], nasty, f"插件参数往返丢失: {nasty!r}")
            self.assertEqual(reloaded["data.sequences"], [nasty, "b"])
            self.assertEqual(reloaded, config)

    def test_rendered_task_roundtrip(self):
        schema = fake_schema()
        original = minimal(task={"name": "demo", "repeats": 7}, data={"sequences": ["00", "01"]})
        original.override(f"{LIO}.max_iterations", 5)
        original["system"] = "../systems/x.yaml"
        del original[LIO]
        self.assertEqual(Config.from_yaml(original.to_yaml(schema), schema), original)

    def test_hand_written_subset(self):
        text = (
            "# 注释\n"
            "task:\n  name: demo#1   # 引号外、空白之后的 # 才是注释\n  repeats: 7\n"
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
                     "task:\n  name: 'x'\n", "task:\n   name: x\n", "task:\n  repeats: \"3\"\n",
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
