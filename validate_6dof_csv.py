#!/usr/bin/env python3
"""
CSV 데이터 검증 및 통계 분석 스크립트
6DOF 포즈 추출 결과를 검증합니다.
"""

import pandas as pd
import numpy as np
import sys
import glob
from pathlib import Path

def validate_6dof_csv(csv_path):
    """CSV 파일 검증"""
    print(f"\n{'='*70}")
    print(f"📊 6DOF CSV Validation Report")
    print(f"{'='*70}\n")

    try:
        df = pd.read_csv(csv_path)
    except FileNotFoundError:
        print(f"❌ CSV 파일을 찾을 수 없습니다: {csv_path}")
        return False
    except Exception as e:
        print(f"❌ CSV 읽기 오류: {e}")
        return False

    print(f"✅ 파일 크기: {Path(csv_path).stat().st_size / 1024:.1f} KB")
    print(f"✅ 총 행 수: {len(df):,}")
    print(f"✅ 컬럼 수: {len(df.columns)}")
    print(f"\n컬럼 목록:")
    for i, col in enumerate(df.columns, 1):
        print(f"  {i:2d}. {col}")

    print(f"\n{'='*70}")
    print(f"📋 데이터 통계")
    print(f"{'='*70}\n")

    # 기본 통계
    print(f"Timestamp:")
    print(f"  시작: {df['timestamp'].min()}")
    print(f"  종료: {df['timestamp'].max()}")
    print(f"  범위: {df['timestamp'].max() - df['timestamp'].min():.2f}초")

    print(f"\nBox ID:")
    print(f"  범위: {df['box_id'].min()} ~ {df['box_id'].max()}")
    print(f"  프레임당 평균 박스: {len(df) / df['timestamp'].nunique():.1f}개")

    # Position 검증
    print(f"\n{'─'*70}")
    print(f"Position (World Frame) - 미터 단위")
    print(f"{'─'*70}")
    for axis in ['x', 'y', 'z']:
        col = f'pos_{axis}'
        print(f"\npos_{axis}:")
        print(f"  Min: {df[col].min():8.3f}m")
        print(f"  Max: {df[col].max():8.3f}m")
        print(f"  Mean: {df[col].mean():8.3f}m")
        print(f"  Std: {df[col].std():8.3f}m")

    # Depth 검증
    print(f"\n{'─'*70}")
    print(f"Depth (D455 Camera)")
    print(f"{'─'*70}")
    print(f"  범위: {df['depth'].min():.3f}m ~ {df['depth'].max():.3f}m")
    print(f"  D455 스펙: 0.6m ~ 6.0m")

    depth_valid = (df['depth'] >= 0.1) & (df['depth'] <= 6.0)
    print(f"  유효 데이터: {depth_valid.sum()}/{len(df)} ({100*depth_valid.sum()/len(df):.1f}%)")

    if not depth_valid.all():
        print(f"  ⚠️  범위 밖 데이터: {(~depth_valid).sum()}개")

    # Quaternion 검증
    print(f"\n{'─'*70}")
    print(f"Quaternion (Orientation)")
    print(f"{'─'*70}")

    quat_norm = np.sqrt(
        df['quat_x']**2 +
        df['quat_y']**2 +
        df['quat_z']**2 +
        df['quat_w']**2
    )

    print(f"  Norm (should be 1.0):")
    print(f"    Min: {quat_norm.min():.6f}")
    print(f"    Max: {quat_norm.max():.6f}")
    print(f"    Mean: {quat_norm.mean():.6f}")
    print(f"    Std: {quat_norm.std():.6f}")

    quat_valid = (quat_norm > 0.99) & (quat_norm < 1.01)
    print(f"\n  정규화된 quaternion: {quat_valid.sum()}/{len(df)} ({100*quat_valid.sum()/len(df):.1f}%)")

    if not quat_valid.all():
        print(f"  ⚠️  비정규화 데이터: {(~quat_valid).sum()}개")
        print(f"    Norm < 0.99: {(quat_norm < 0.99).sum()}개")
        print(f"    Norm > 1.01: {(quat_norm > 1.01).sum()}개")

    # Size 검증
    print(f"\n{'─'*70}")
    print(f"Size (3D Bounding Box) - 미터 단위")
    print(f"{'─'*70}")

    for axis in ['x', 'y', 'z']:
        col = f'size_{axis}'
        positive = (df[col] > 0).sum()
        print(f"\nsize_{axis}:")
        print(f"  Min: {df[col].min():.4f}m")
        print(f"  Max: {df[col].max():.4f}m")
        print(f"  Mean: {df[col].mean():.4f}m")
        print(f"  Positive: {positive}/{len(df)} ({100*positive/len(df):.1f}%)")

    # Confidence 검증
    print(f"\n{'─'*70}")
    print(f"Confidence (0.0 ~ 1.0)")
    print(f"{'─'*70}")
    print(f"  Min: {df['confidence'].min():.3f}")
    print(f"  Max: {df['confidence'].max():.3f}")
    print(f"  Mean: {df['confidence'].mean():.3f}")
    print(f"  Median: {df['confidence'].median():.3f}")

    conf_valid = (df['confidence'] >= 0.0) & (df['confidence'] <= 1.0)
    print(f"\n  유효 범위: {conf_valid.sum()}/{len(df)} ({100*conf_valid.sum()/len(df):.1f}%)")

    # 종합 검증
    print(f"\n{'='*70}")
    print(f"✅ 최종 검증")
    print(f"{'='*70}\n")

    all_valid = depth_valid.all() and quat_valid.all() and conf_valid.all()

    checks = [
        ("CSV 파일 존재", True, "✅"),
        ("데이터 행 개수", len(df) > 0, "✅" if len(df) > 0 else "❌"),
        ("Depth 범위 (0.1~6.0m)", depth_valid.all(), "✅" if depth_valid.all() else "⚠️"),
        ("Quaternion 정규화", quat_valid.all(), "✅" if quat_valid.all() else "⚠️"),
        ("Confidence 범위", conf_valid.all(), "✅" if conf_valid.all() else "⚠️"),
        ("Size 양수", (df[['size_x', 'size_y', 'size_z']] > 0).all().all(), "✅"),
    ]

    for check_name, result, symbol in checks:
        print(f"{symbol} {check_name}: {'PASS' if result else 'FAIL'}")

    print(f"\n{'='*70}")
    if all_valid:
        print(f"🎉 모든 검증 통과! 데이터가 유효합니다.\n")
        return True
    else:
        print(f"⚠️  일부 검증 실패. 위 내용을 확인하세요.\n")
        return False

if __name__ == '__main__':
    # 최신 CSV 파일 찾기
    pattern = Path.home() / 'ros2_ws/runs/segment/predict*/box_6dof.csv'
    csv_files = sorted(glob.glob(str(pattern)))

    if not csv_files:
        print("\n❌ CSV 파일을 찾을 수 없습니다.")
        print(f"예상 위치: {pattern}")
        sys.exit(1)

    # 가장 최신 파일 사용
    csv_path = csv_files[-1]
    print(f"\n📁 CSV 파일: {csv_path}")

    validate_6dof_csv(csv_path)
