# Changelog

## v0.2.0 — 2026-11-05

### Licensing

- Adopt PolyForm Noncommercial License 1.0.0 for the current offering of
  original NavCraft material. Commercial uses outside the license's
  permissions require separate written authorization.
- Update all 13 ROS package manifests to version `0.2.0` and SPDX identifier
  `PolyForm-Noncommercial-1.0.0`.
- Add licensing guidance, notices, and the original MIT text for historical
  reference. Update both README entry points and release badges.
- Preserve the MIT-licensed `v0.1` tag and all rights previously granted
  under MIT. Third-party material remains subject to its own terms.

This release changes licensing, version metadata, and documentation. It does
not change navigation behavior, algorithms, interfaces, maps, or simulation
parameters. Existing runtime validation is documented in
[validation.md](docs/validation.md); no new runtime test results are claimed
for this licensing change.

**中文：** 自本版本起，当前 NavCraft 原创内容采用 PolyForm 非商业许可；超出许可允许范围的商业使用需另行取得书面授权。13 个包的版本与许可声明已同步。旧 `v0.1` 的 MIT 授权及第三方许可证不受影响。本次不修改运行功能。

## v0.1 — 2026-11-05

- Initial 13-package ROS 2 Jazzy / Gazebo Harmonic navigation and control
  simulation platform, with warehouse missions, mapping, tracking, velocity
  protection, and validation tools.
- Published under the [MIT License](licenses/MIT-v0.1.txt).

See [licensing details](docs/licensing.md) for the version boundary and
commercial licensing inquiries.
