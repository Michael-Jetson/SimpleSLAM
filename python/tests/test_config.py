"""Python 侧配置对象：渲染、校验、解析子集、哈希稳定性。"""

import unittest

from simpleslam.config import Config, ConfigError
from simpleslam.schema import Field, Schema


def fake_schema() -> Schema:
    return Schema(
        [
            Field("name", "string", "default", "实验名", False),
            Field("seed", "int", "0", "随机种子", False),
            Field("runtime.offline", "bool", "true", "离线步进", False),
            Field("runtime.worker_threads", "int", "2", "线程数", False),
            Field("odometry.max_iterations", "int", "30", "迭代上限", True),
            Field("odometry.voxel_size_m", "double", "0.5", "体素边长", True),
        ]
    )


class ConfigTest(unittest.TestCase):
    def test_only_explicit_items_are_written(self):
        config = Config(name="demo")
        config["odometry.max_iterations"] = 40
        text = config.to_yaml(fake_schema())
        self.assertIn('name: "demo"', text)  # 字符串一律加引号
        self.assertIn("  max_iterations: 40", text)
        self.assertNotIn("voxel_size_m", text)  # 没改的不抄进配置文件

    def test_unknown_key_is_an_error(self):
        config = Config()
        config["odometry.max_iteration"] = 40  # 少一个 s
        with self.assertRaises(ConfigError):
            config.validate(fake_schema())

    def test_type_is_not_coerced(self):
        schema = fake_schema()
        with self.assertRaises(ConfigError):
            Config(**{"runtime.offline": 1}).validate(schema)
        with self.assertRaises(ConfigError):
            Config(**{"odometry.max_iterations": "40"}).validate(schema)
        with self.assertRaises(ConfigError):
            Config(**{"odometry.max_iterations": 1.5}).validate(schema)
        # 整数喂给 double 字段可以：那是拓宽，不是改变含义
        Config(**{"odometry.voxel_size_m": 1}).validate(schema)

    def test_canonical_fills_defaults_in_schema_order(self):
        schema = fake_schema()
        text = Config(name="demo").canonical(schema)
        # 字符串一律加引号：不加引号的导出不可逆（`a # b` 会变成 `a`）
        self.assertEqual(
            text,
            'name: "demo"\nseed: 0\n'
            "\nruntime:\n  offline: true\n  worker_threads: 2\n"
            "\nodometry:\n  max_iterations: 30\n  voxel_size_m: 0.5\n",
        )

    def test_hash_depends_only_on_content(self):
        schema = fake_schema()
        a = Config(name="demo", **{"odometry.max_iterations": 40})
        b = Config(**{"odometry.max_iterations": 40}, name="demo")
        self.assertEqual(a.hash(schema), b.hash(schema))

        c = Config(name="demo", **{"odometry.max_iterations": 41})
        self.assertNotEqual(a.hash(schema), c.hash(schema))
        # 显式写成与默认值相同也不改变哈希：哈希看的是生效值
        d = Config(name="demo", **{"odometry.max_iterations": 40, "odometry.voxel_size_m": 0.5})
        self.assertEqual(a.hash(schema), d.hash(schema))

    def test_canonical_is_lossless_for_nasty_strings(self):
        schema = fake_schema()
        for nasty in ('a # b', 'x: y', '"quoted"', 'tail  ', 'back\\slash', 'line\nbreak',
                      'true', '42'):
            config = Config(name=nasty)
            reloaded = Config.from_yaml(config.canonical(schema), schema)
            self.assertEqual(reloaded["name"], nasty, f"往返丢失: {nasty!r}")
            self.assertEqual(reloaded.hash(schema), config.hash(schema))

    def test_yaml_subset_roundtrip(self):
        schema = fake_schema()
        original = Config(name="demo", seed=7)
        original["odometry.voxel_size_m"] = 0.25
        loaded = Config.from_yaml(original.to_yaml(schema), schema)
        self.assertEqual(loaded["name"], "demo")
        self.assertEqual(loaded["seed"], 7)
        self.assertEqual(loaded["odometry.voxel_size_m"], 0.25)

    def test_yaml_subset_rejects_deeper_nesting(self):
        with self.assertRaises(ConfigError):
            Config.from_yaml("a:\n  b:\n    c: 1\n")

    def test_update_by_section(self):
        config = Config()
        config.update(odometry={"max_iterations": 12}, seed=3)
        self.assertEqual(config["odometry.max_iterations"], 12)
        self.assertEqual(config["seed"], 3)


if __name__ == "__main__":
    unittest.main()
