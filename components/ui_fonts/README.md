# Mesh fonts

`mesh_text_22.c` supplies Latin text, combining accents and punctuation at the
existing 22 px message size. It falls back to `mesh_unicode_18.c`, the bundled
18 px Cascadia/Noto Emoji bitmap font, for monochrome emoji. The compact font
also serves node names and preset labels. Both fonts must be linked in the
firmware and simulator.

The exact conversion options are recorded at the top of each generated C file.
After regenerating `mesh_text_22.c`, declare `extern const lv_font_t
mesh_unicode_18;` and set its `.fallback` to `&mesh_unicode_18`. The existing
Unicode asset was generated from Cascadia Code and Noto Emoji; its original
`/tmp/NotoEmoji.ttf` source path is not bundled. Both font families use the
SIL Open Font License.

This does not add emoji sequence shaping or color rendering. Multi-code-point
flags, skin-tone modifiers and ZWJ sequences may appear as separate glyphs.
Simulator navigation tests check actual glyph resolution for German letters
and representative emoji in both sizes. Mesh mock messages exercise them
visually; capture with `UWATCH_SIM_SCREEN=mesh` and `UWATCH_SIM_SHOT`.
