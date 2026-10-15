"""
軌跡プロット GUI ツール (拡張版)

t検定で生成された軌跡データ(JSON)を読み込み、
GUIで各種パラメータを調整してPDF/PNGを保存します。

機能:
- 軌道の表示/非表示
- 線の太さ
- X/Y軸範囲
- タイトル
- 図サイズ (幅, 高さ)
- DPI
- アスペクト比 (等比/自由)
- 凡例の位置
- 軌道の色
"""

import json
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Slider, Button, TextBox, RadioButtons
from matplotlib.colors import to_hex
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, colorchooser
import sys

# ============================================
# 1. ファイル選択
# ============================================
def select_file():
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="解析するJSONファイルを選択してください",
        filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
    )
    root.destroy()
    return file_path

json_path = select_file()
if not json_path:
    sys.exit()

try:
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
except Exception as e:
    print(f"エラー: {e}")
    sys.exit()

# ============================================
# 2. 初期設定
# ============================================
current_figsize = [12, 8]  # [幅, 高さ]
current_dpi = 150
current_aspect_equal = True

fig, ax = plt.subplots(figsize=tuple(current_figsize))
plt.subplots_adjust(left=0.18, bottom=0.32, right=0.85)

# ============================================
# 3. 軌道プロット
# ============================================
lines = []
line_keys = []  # 'real', 'initial', 'optimized'

# Real
l0, = ax.plot(data['real']['x'], data['real']['y'], 'k--', 
              label='Real', alpha=0.7, linewidth=1.5)
lines.append(l0)
line_keys.append('real')

# Initial
if 'initial' in data:
    rmse_init = data['initial'].get('rmse', 'N/A')
    l1, = ax.plot(data['initial']['x'], data['initial']['y'], 'r-', 
                  label=f"Initial (RMSE:{rmse_init:.4f}m)" if isinstance(rmse_init, float) else f"Initial",
                  alpha=0.7, linewidth=1.5)
    lines.append(l1)
    line_keys.append('initial')

# Optimized
if 'optimized' in data:
    rmse_opt = data['optimized'].get('rmse', 'N/A')
    l2, = ax.plot(data['optimized']['x'], data['optimized']['y'], 'b-', 
                  label=f"Optimized (RMSE:{rmse_opt:.4f}m)" if isinstance(rmse_opt, float) else f"Optimized",
                  alpha=0.8, linewidth=1.5)
    lines.append(l2)
    line_keys.append('optimized')

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_title(f"Trajectory: {json_path.split('/')[-1]}")
ax.axis('equal')
ax.grid(True, linestyle='--', alpha=0.5)
legend = ax.legend(loc='upper right', fontsize='small')

initial_xlim = ax.get_xlim()
initial_ylim = ax.get_ylim()

# ============================================
# 4. GUI: 表示/非表示 チェックボックス
# ============================================
rax_check = plt.axes([0.02, 0.65, 0.12, 0.15])
check = CheckButtons(rax_check, [l.get_label()[:15] for l in lines], [True]*len(lines))

def toggle_visibility(label):
    for i, l in enumerate(lines):
        if l.get_label()[:15] == label:
            l.set_visible(not l.get_visible())
    plt.draw()
check.on_clicked(toggle_visibility)

# ============================================
# 5. GUI: 線の太さスライダー
# ============================================
ax_width = plt.axes([0.25, 0.20, 0.25, 0.025])
s_width = Slider(ax_width, 'Line Width', 0.5, 5.0, valinit=1.5)

def update_width(val):
    for l in lines:
        l.set_linewidth(s_width.val)
    fig.canvas.draw_idle()
s_width.on_changed(update_width)

# ============================================
# 6. GUI: X/Y軸範囲
# ============================================
ax_xmin = plt.axes([0.1, 0.14, 0.07, 0.03])
ax_xmax = plt.axes([0.19, 0.14, 0.07, 0.03])
ax_ymin = plt.axes([0.33, 0.14, 0.07, 0.03])
ax_ymax = plt.axes([0.42, 0.14, 0.07, 0.03])

txt_xmin = TextBox(ax_xmin, 'X:', initial=f"{initial_xlim[0]:.1f}")
txt_xmax = TextBox(ax_xmax, '-', initial=f"{initial_xlim[1]:.1f}")
txt_ymin = TextBox(ax_ymin, 'Y:', initial=f"{initial_ylim[0]:.1f}")
txt_ymax = TextBox(ax_ymax, '-', initial=f"{initial_ylim[1]:.1f}")

# ============================================
# 7. GUI: タイトル
# ============================================
ax_title = plt.axes([0.1, 0.08, 0.4, 0.03])
txt_title = TextBox(ax_title, 'Title:', initial=ax.get_title())

# ============================================
# 8. GUI: 図サイズ (幅, 高さ)
# ============================================
ax_figw = plt.axes([0.62, 0.20, 0.06, 0.03])
ax_figh = plt.axes([0.72, 0.20, 0.06, 0.03])
txt_figw = TextBox(ax_figw, 'Fig W:', initial=str(current_figsize[0]))
txt_figh = TextBox(ax_figh, 'H:', initial=str(current_figsize[1]))

# ============================================
# 9. GUI: DPI
# ============================================
ax_dpi = plt.axes([0.62, 0.14, 0.06, 0.03])
txt_dpi = TextBox(ax_dpi, 'DPI:', initial=str(current_dpi))

# ============================================
# 10. GUI: アスペクト比
# ============================================
ax_aspect = plt.axes([0.86, 0.55, 0.12, 0.12])
radio_aspect = RadioButtons(ax_aspect, ('Equal', 'Auto'), active=0)

def set_aspect(label):
    global current_aspect_equal
    if label == 'Equal':
        ax.set_aspect('equal', adjustable='box')
        current_aspect_equal = True
    else:
        ax.set_aspect('auto')
        current_aspect_equal = False
    plt.draw()
radio_aspect.on_clicked(set_aspect)

# ============================================
# 11. GUI: 凡例の位置
# ============================================
ax_legend = plt.axes([0.86, 0.35, 0.12, 0.18])
legend_positions = ['upper right', 'upper left', 'lower left', 'lower right', 'center']
radio_legend = RadioButtons(ax_legend, legend_positions, active=0)

def set_legend_pos(label):
    ax.legend(loc=label, fontsize='small')
    plt.draw()
radio_legend.on_clicked(set_legend_pos)

# ============================================
# 12. GUI: 軌道の色変更ボタン
# ============================================
color_buttons = []
for i, key in enumerate(line_keys):
    ax_color = plt.axes([0.02, 0.45 - i*0.06, 0.12, 0.04])
    btn = Button(ax_color, f'{key[:8]} color', color=lines[i].get_color(), hovercolor='lightgray')
    color_buttons.append((btn, i))

def make_color_callback(line_idx):
    def callback(event):
        root = tk.Tk()
        root.withdraw()
        color = colorchooser.askcolor(title=f"Choose color for {line_keys[line_idx]}")
        root.destroy()
        if color[1]:
            lines[line_idx].set_color(color[1])
            color_buttons[line_idx][0].color = color[1]
            plt.draw()
    return callback

for btn, idx in color_buttons:
    btn.on_clicked(make_color_callback(idx))

# ============================================
# 13. GUI: 適用ボタン
# ============================================
ax_apply = plt.axes([0.55, 0.08, 0.08, 0.04])
btn_apply = Button(ax_apply, 'Apply', color='lightgreen', hovercolor='green')

def apply_settings(event):
    global current_dpi, current_figsize
    try:
        # 軸範囲
        xmin, xmax = float(txt_xmin.text), float(txt_xmax.text)
        ymin, ymax = float(txt_ymin.text), float(txt_ymax.text)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        
        # タイトル
        ax.set_title(txt_title.text)
        
        # DPI (保存時に使用)
        current_dpi = int(txt_dpi.text)
        
        # 図サイズ (保存時に使用)
        current_figsize = [float(txt_figw.text), float(txt_figh.text)]
        
        # アスペクト比を再適用
        if current_aspect_equal:
            ax.set_aspect('equal', adjustable='box')
        else:
            ax.set_aspect('auto')
        
        plt.draw()
        print("Settings applied.")
    except ValueError as e:
        print(f"入力エラー: {e}")

btn_apply.on_clicked(apply_settings)

# ============================================
# 14. GUI: PDF保存ボタン
# ============================================
ax_save_pdf = plt.axes([0.72, 0.08, 0.08, 0.04])
btn_save_pdf = Button(ax_save_pdf, 'Save PDF', color='lightblue', hovercolor='blue')

def save_pdf(event):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_fn = f"trajectory_plot_{timestamp}.pdf"
    
    # 一時的に図サイズを変更して保存
    old_size = fig.get_size_inches()
    fig.set_size_inches(current_figsize[0], current_figsize[1])
    plt.savefig(output_fn, format='pdf', bbox_inches='tight', dpi=current_dpi)
    fig.set_size_inches(old_size)
    
    print(f"Saved: {output_fn} (DPI={current_dpi}, Size={current_figsize})")

btn_save_pdf.on_clicked(save_pdf)

# ============================================
# 15. GUI: PNG保存ボタン
# ============================================
ax_save_png = plt.axes([0.82, 0.08, 0.08, 0.04])
btn_save_png = Button(ax_save_png, 'Save PNG', color='lightyellow', hovercolor='yellow')

def save_png(event):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_fn = f"trajectory_plot_{timestamp}.png"
    
    old_size = fig.get_size_inches()
    fig.set_size_inches(current_figsize[0], current_figsize[1])
    plt.savefig(output_fn, format='png', bbox_inches='tight', dpi=current_dpi)
    fig.set_size_inches(old_size)
    
    print(f"Saved: {output_fn} (DPI={current_dpi}, Size={current_figsize})")

btn_save_png.on_clicked(save_png)

# ============================================
# 表示
# ============================================
plt.show()