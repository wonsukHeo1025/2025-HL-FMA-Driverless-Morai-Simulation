#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def plot_csv_data(csv_file):
    """CSV 데이터 시각화 - 중요 필드만"""
    
    # CSV 읽기
    df = pd.read_csv(csv_file)
    print(f"데이터 크기: {len(df)} rows")
    
    # 시간 인덱스 생성 (50Hz 가정)
    df['time'] = df.index * 0.02  # 20ms 간격
    
    # 중요 필드 선택
    fields_to_plot = [
        ('true_velocity_x', 'Velocity X (m/s)', 'blue'),
        ('steer_cmd', 'Steering Command (rad)', 'green'),
        ('true_wheel_angle', 'True Wheel Angle (rad)', 'orange'),
        ('imu_accel_y', 'IMU Lateral Accel (m/s²)', 'red'),
        ('yaw_rate', 'Yaw Rate (rad/s)', 'purple') if 'yaw_rate' in df.columns else None,
    ]
    
    # None 제거
    fields_to_plot = [f for f in fields_to_plot if f is not None]
    
    # 서브플롯 생성
    fig, axes = plt.subplots(len(fields_to_plot), 1, figsize=(12, 3*len(fields_to_plot)), sharex=True)
    if len(fields_to_plot) == 1:
        axes = [axes]
    
    # 각 필드 플로팅
    for idx, (field, label, color) in enumerate(fields_to_plot):
        if field in df.columns:
            axes[idx].plot(df['time'], df[field], color=color, linewidth=0.5)
            axes[idx].set_ylabel(label)
            axes[idx].grid(True, alpha=0.3)
            
            # 이상치 감지 (3 시그마)
            mean = df[field].mean()
            std = df[field].std()
            outliers = df[(df[field] > mean + 3*std) | (df[field] < mean - 3*std)]
            
            if len(outliers) > 0:
                axes[idx].scatter(outliers['time'], outliers[field], 
                                color='red', s=20, marker='x', label=f'이상치 {len(outliers)}개')
                axes[idx].legend()
                print(f"⚠️  {field}: {len(outliers)}개 이상치 감지됨")
    
    axes[-1].set_xlabel('Time (s)')
    
    # 제목 설정
    filename = os.path.basename(csv_file)
    plt.suptitle(f'{filename}\n데이터 무결성 검사', fontsize=12)
    
    plt.tight_layout()
    plt.show()
    
    # 데이터 연속성 체크
    print("\n=== 데이터 연속성 검사 ===")
    
    # 속도 변화율 체크
    if 'true_velocity_x' in df.columns:
        df['velocity_diff'] = df['true_velocity_x'].diff()
        max_diff = df['velocity_diff'].abs().max()
        if max_diff > 5.0:  # 5 m/s 이상 급변
            print(f"⚠️  속도 급변 감지: 최대 {max_diff:.2f} m/s")
            problem_idx = df['velocity_diff'].abs().idxmax()
            print(f"   위치: {problem_idx}행 (시간: {df.loc[problem_idx, 'time']:.2f}초)")
    
    # 조향각 변화율 체크  
    if 'steer_cmd' in df.columns:
        df['steer_diff'] = df['steer_cmd'].diff()
        max_steer_diff = df['steer_diff'].abs().max()
        if max_steer_diff > 0.5:  # 0.5 rad 이상 급변
            print(f"⚠️  조향 급변 감지: 최대 {max_steer_diff:.3f} rad")
            problem_idx = df['steer_diff'].abs().idxmax()
            print(f"   위치: {problem_idx}행 (시간: {df.loc[problem_idx, 'time']:.2f}초)")
    
    print("\n✅ 검사 완료")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        # 파일 지정 안하면 최신 파일 자동 선택
        data_dir = os.path.dirname(os.path.abspath(__file__))
        csv_files = [f for f in os.listdir(data_dir) if f.endswith('.csv')]
        
        if not csv_files:
            print("CSV 파일이 없습니다.")
            sys.exit(1)
        
        # 최신 파일 선택
        csv_files.sort()
        csv_file = os.path.join(data_dir, csv_files[-1])
        print(f"최신 파일 선택: {csv_files[-1]}")
    else:
        csv_file = sys.argv[1]
        
    if not os.path.exists(csv_file):
        print(f"파일을 찾을 수 없음: {csv_file}")
        sys.exit(1)
    
    plot_csv_data(csv_file)