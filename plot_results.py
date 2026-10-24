"""
plot_results.py
---------------
Makale Figür 30 ve 33'e benzer şekilde:
1. Robotun Kat Ettiği Yol ve Hedef Noktaları (2D Trajectory)
2. Adımlara Göre Konum Hatası (Position Error vs Steps)
3. Sol ve Sağ Tekerlek Hız Profilleri (Motor Velocities)
4. Hedef Açı/Yönelim Hatası (Heading Error)
grafiklerini harici ağır kütüphanelere bağımlı olmadan doğrudan csv okuyarak
matplotlib ile oluşturup 'navigation_performance.png' olarak kaydeder.
"""

import csv
import matplotlib.pyplot as plt

def generate_plots(csv_file="navigation_results.csv", output_img="navigation_performance.png"):
    steps = []
    x = []
    y = []
    pos_error = []
    heading_error = []
    v_left = []
    v_right = []
    
    with open(csv_file, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row["step"]))
            x.append(float(row["x"]))
            y.append(float(row["y"]))
            pos_error.append(float(row["pos_error"]))
            heading_error.append(float(row["heading_error"]) * (180.0 / 3.14159))
            v_left.append(float(row["v_left"]))
            v_right.append(float(row["v_right"]))
            
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Pioneer P3-DX Otonom Coklu Hedef Bulanik Navigasyon Sonuclari\n(Sensors 2025 Makale Metodolojisi)", fontsize=14, fontweight='bold')
    
    # 1. 2D Rota
    ax1 = axes[0, 0]
    ax1.plot(x, y, 'b-', label='Robot Gercek Rotasi', linewidth=2)
    ax1.scatter([x[0]], [y[0]], color='green', s=120, label='Baslangic / Depot', zorder=5)
    ax1.set_title("2D Navigasyon Rotasi")
    ax1.set_xlabel("X (metre)")
    ax1.set_ylabel("Y (metre)")
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend()
    
    # 2. Position Error vs Step (Makale Figür 30)
    ax2 = axes[0, 1]
    ax2.plot(steps, pos_error, 'r-', linewidth=1.8)
    ax2.set_title("Hedefe Olan Konum Hatasi (Position Error vs Steps)")
    ax2.set_xlabel("Adim Sayisi (Steps)")
    ax2.set_ylabel("Konum Hatasi (metre)")
    ax2.grid(True, linestyle='--', alpha=0.6)
    
    # 3. Tekerlek Hızları
    ax3 = axes[1, 0]
    ax3.plot(steps, v_left, 'purple', label='Sol Tekerlek Hizi (v_L)', alpha=0.8)
    ax3.plot(steps, v_right, 'orange', label='Sag Tekerlek Hizi (v_R)', alpha=0.8)
    ax3.set_title("Tekerlek Hiz Profili (rad/s)")
    ax3.set_xlabel("Adim Sayisi (Steps)")
    ax3.set_ylabel("Acsal Hiz (rad/s)")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend()
    
    # 4. Yönelim Hatası (Heading Error)
    ax4 = axes[1, 1]
    ax4.plot(steps, heading_error, 'g-', linewidth=1.5)
    ax4.set_title("Hedef Yonelim Hatasi (Heading Error vs Steps)")
    ax4.set_xlabel("Adim Sayisi (Steps)")
    ax4.set_ylabel("Aci Hatasi (Derece)")
    ax4.grid(True, linestyle='--', alpha=0.6)
    
    plt.tight_layout()
    plt.savefig(output_img, dpi=300)
    print(f"Performans grafikleri basariyla '{output_img}' dosyasina kaydedildi.")

if __name__ == "__main__":
    generate_plots()
