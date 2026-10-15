"""
Streamlitによるリアルタイムダッシュボード

最適化の進捗、軌跡比較、パラメータ重要度をリアルタイムで表示します。

使用方法:
    streamlit run dashboard.py -- --study-name awsim_optimization --db-path sqlite:///optuna_study.db
"""

import streamlit as st
import optuna
import numpy as np
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import pandas as pd
import json
import time
from pathlib import Path
import argparse
from typing import Optional, Dict

# ページ設定
st.set_page_config(
    page_title="AWSIM Optimizer Dashboard",
    page_icon="🚗",
    layout="wide",
    initial_sidebar_state="expanded"
)


def load_study(study_name: str, storage_path: str) -> Optional[optuna.Study]:
    """Optunaのstudyを読み込み"""
    try:
        study = optuna.load_study(study_name=study_name, storage=storage_path)
        # デバッグ: 読み込んだStudyの情報を表示
        st.sidebar.success(f"✓ Study: {len(study.trials)} trials")
        return study
    except Exception as e:
        st.sidebar.error(f"Study読み込みエラー: {e}")
        return None


def load_trial_history(results_path: str) -> Optional[list]:
    """試行履歴をJSONから読み込み"""
    try:
        with open(results_path, 'r') as f:
            data = json.load(f)
        return data.get("trial_history", [])
    except Exception:
        return None


def plot_optimization_history(study: optuna.Study, clip_outliers: bool = True) -> go.Figure:
    """最適化履歴をPlotlyでプロット
    
    Args:
        study: Optuna Study
        clip_outliers: True の場合、Y軸を95パーセンタイルでクリップして外れ値を除外
    """
    trials = [t for t in study.trials if t.value is not None]
    
    if len(trials) == 0:
        fig = go.Figure()
        fig.add_annotation(text="No completed trials yet", 
                          xref="paper", yref="paper", x=0.5, y=0.5)
        return fig
    
    trial_numbers = [t.number for t in trials]
    values = [t.value for t in trials]
    
    # ベストスコアの推移
    best_values = []
    current_best = float('inf')
    for v in values:
        current_best = min(current_best, v)
        best_values.append(current_best)
    
    fig = go.Figure()
    
    # 全試行
    fig.add_trace(go.Scatter(
        x=trial_numbers, y=values,
        mode='markers',
        name='Trial Score',
        marker=dict(size=8, opacity=0.6)
    ))
    
    # ベストスコア
    fig.add_trace(go.Scatter(
        x=trial_numbers, y=best_values,
        mode='lines',
        name='Best Score',
        line=dict(color='red', width=2)
    ))
    
    # Y軸の範囲を調整（外れ値対策）
    layout_update = {
        "title": "Optimization History",
        "xaxis_title": "Trial Number",
        "yaxis_title": "Score (RMSE Total)",
        "hovermode": 'x unified'
    }
    
    if clip_outliers and len(values) >= 3:
        # 95パーセンタイルでY軸上限をクリップ
        p95 = np.percentile(values, 95)
        y_min = min(best_values) * 0.9  # ベスト値の少し下
        y_max = p95 * 1.1  # 95パーセンタイルの少し上
        
        # ベスト値とp95が近い場合は少しマージンを追加
        if y_max - y_min < 0.1:
            y_max = y_min + 0.1
        
        layout_update["yaxis"] = dict(range=[max(0, y_min), y_max])
    
    fig.update_layout(**layout_update)
    
    return fig



def plot_param_importance(study: optuna.Study) -> Optional[go.Figure]:
    """パラメータ重要度をプロット"""
    try:
        importance = optuna.importance.get_param_importances(study)
        
        if len(importance) == 0:
            return None
        
        # ソート
        sorted_items = sorted(importance.items(), key=lambda x: x[1], reverse=True)
        names = [item[0] for item in sorted_items]
        values = [item[1] for item in sorted_items]
        
        fig = go.Figure(go.Bar(
            x=values,
            y=names,
            orientation='h',
            marker_color='steelblue'
        ))
        
        fig.update_layout(
            title="Parameter Importance",
            xaxis_title="Importance",
            yaxis=dict(autorange="reversed"),
            height=max(400, len(names) * 25)
        )
        
        return fig
    except Exception as e:
        st.warning(f"パラメータ重要度の計算にはより多くの試行が必要です: {e}")
        return None


def plot_parallel_coordinates(study: optuna.Study) -> Optional[go.Figure]:
    """パラレル座標プロット"""
    trials = [t for t in study.trials if t.value is not None]
    
    if len(trials) < 3:
        return None
    
    # データフレームを作成
    data = []
    for t in trials:
        row = t.params.copy()
        row['score'] = t.value
        data.append(row)
    
    df = pd.DataFrame(data)
    
    # スコアで色付け
    fig = px.parallel_coordinates(
        df,
        color='score',
        color_continuous_scale=px.colors.diverging.Tealrose,
        color_continuous_midpoint=df['score'].median()
    )
    
    fig.update_layout(
        title="Parallel Coordinates (Parameters vs Score)",
        height=500
    )
    
    return fig


def plot_scenario_scores(trial_history: list) -> Optional[go.Figure]:
    """シナリオ別スコアの推移"""
    if not trial_history:
        return None
    
    # シナリオ名を収集
    all_scenarios = set()
    for trial in trial_history:
        if 'scenario_scores' in trial:
            all_scenarios.update(trial['scenario_scores'].keys())
    
    if not all_scenarios:
        return None
    
    fig = go.Figure()
    
    trial_numbers = [t['trial_number'] for t in trial_history]
    
    for scenario in sorted(all_scenarios):
        scores = []
        for trial in trial_history:
            score = trial.get('scenario_scores', {}).get(scenario, None)
            scores.append(score)
        
        fig.add_trace(go.Scatter(
            x=trial_numbers,
            y=scores,
            mode='lines+markers',
            name=scenario,
            marker=dict(size=6)
        ))
    
    fig.update_layout(
        title="Scenario-wise RMSE",
        xaxis_title="Trial Number",
        yaxis_title="RMSE [m]",
        hovermode='x unified'
    )
    
    return fig


def display_best_params(study: optuna.Study):
    """ベストパラメータを表示"""
    if study.best_trial is None:
        st.info("まだ完了した試行がありません")
        return
    
    st.subheader(f"🏆 Best Score: {study.best_value:.4f}")
    
    params = study.best_params
    
    # カテゴリ別に分類
    categories = {
        "物理": ["skidding_cancel_rate", "front_wheel_skidding_rate", "forward_slip_multiplier"],
        "遅延": ["acceleration_time_constant", "steer_tire_angle_time_constant"],
        "PID": ["wheel_kp", "wheel_ki", "wheel_kd", "wheel_integral_clamp"],
        "抵抗": ["air_drag", "rolling_resistance"],
        "重心": ["com_offset_x", "com_offset_y", "com_offset_z"],
        "減速": ["deceleration_curve_scale"]
    }
    
    cols = st.columns(3)
    col_idx = 0
    
    for cat_name, param_names in categories.items():
        with cols[col_idx % 3]:
            st.markdown(f"**{cat_name}**")
            for pname in param_names:
                if pname in params:
                    st.text(f"  {pname}: {params[pname]:.4f}")
        col_idx += 1


def main():
    st.title("🚗 AWSIM Parameter Optimizer Dashboard")
    
    # コマンドライン引数を解析（Streamlitの -- 以降）
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument("--study-name", type=str, default="awsim_optimization")
    parser.add_argument("--db-path", type=str, default="sqlite:///optuna_study.db")
    parser.add_argument("--results-json", type=str, default="optimization_results.json")
    
    # Streamlitの引数を除外して解析
    try:
        args, _ = parser.parse_known_args()
    except:
        args = argparse.Namespace(
            study_name="awsim_optimization",
            db_path="sqlite:///optuna_study.db",
            results_json="optimization_results.json"
        )
    
    # コマンドライン引数をデフォルト値として使用
    default_study_name = args.study_name
    default_db_path = args.db_path
    default_results_json = args.results_json
    
    # サイドバー設定（コマンドライン引数をデフォルト値として使用）
    st.sidebar.header("Settings")
    
    study_name = st.sidebar.text_input("Study Name", value=default_study_name, key="study_name_input")
    db_path_input = st.sidebar.text_input("Database Path", value=default_db_path, key="db_path_input")
    results_path_input = st.sidebar.text_input("Results JSON", value=default_results_json, key="results_path_input")
    
    # 実際に使用するパスを決定
    results_path = results_path_input
    db_path = db_path_input
    
    # results_pathが見つからない場合、親ディレクトリも探す
    if not Path(results_path).exists():
        parent_path = Path(__file__).parent.parent / results_path_input
        if parent_path.exists():
            results_path = str(parent_path)
    
    # db_pathも同様に親ディレクトリを探す
    db_file = db_path_input.replace("sqlite:///", "")
    if not Path(db_file).exists():
        parent_db = Path(__file__).parent.parent / db_file
        if parent_db.exists():
            db_path = f"sqlite:///{parent_db}"
    
    # 現在読み込み中のパスを表示
    st.sidebar.caption(f"📂 DB: {db_path}")
    st.sidebar.caption(f"📄 JSON: {results_path}")
    
    auto_refresh = st.sidebar.checkbox("Auto Refresh", value=True)
    refresh_interval = st.sidebar.slider("Refresh Interval (sec)", 3, 60, 5)
    
    if st.sidebar.button("🔄 Refresh Now"):
        st.rerun()
    
    # Study読み込み（パスが変わるたびに再読み込み）
    study = load_study(study_name, db_path)
    trial_history = load_trial_history(results_path)
    
    if study is None:
        st.warning("Studyが見つかりません。最適化を開始してください。")
        st.code(f"""
# 最適化の開始方法:
cd optimizer
python3 -m whill_optimizer optimize --trials 100
        """)
        return
    
    # メトリクス表示
    col1, col2, col3, col4, col5 = st.columns(5)
    
    completed_trials = len([t for t in study.trials if t.value is not None])
    pruned_trials = len([t for t in study.trials if t.state == optuna.trial.TrialState.PRUNED])
    
    with col1:
        st.metric("Total Trials", len(study.trials))
    with col2:
        st.metric("Completed", completed_trials)
    with col3:
        st.metric("Pruned ⚡", pruned_trials, help="早期打ち切りされた試行数")
    with col4:
        if study.best_value is not None:
            st.metric("Best Score", f"{study.best_value:.4f}")
        else:
            st.metric("Best Score", "N/A")
    with col5:
        if completed_trials > 0:
            avg_score = np.mean([t.value for t in study.trials if t.value is not None])
            st.metric("Avg Score", f"{avg_score:.4f}")
        else:
            st.metric("Avg Score", "N/A")
    
    st.divider()
    
    # タブ表示
    tab1, tab2, tab3, tab4, tab5 = st.tabs(["📈 History", "🗺️ Trajectories", "📊 Analysis", "🎯 Best Params", "📋 Trials"])
    
    with tab1:
        # ズーム調整UI
        values = [t.value for t in study.trials if t.value is not None]
        if values:
            col_zoom1, col_zoom2, col_zoom3 = st.columns([1, 1, 2])
            with col_zoom1:
                clip_outliers = st.checkbox("外れ値を除外", value=True, help="Y軸を95パーセンタイルでクリップ")
            with col_zoom2:
                custom_range = st.checkbox("Y軸範囲を手動設定", value=False)
            
            y_range = None
            if custom_range:
                y_min_default = min(values) * 0.9
                y_max_default = np.percentile(values, 95) * 1.2
                with col_zoom3:
                    y_range = st.slider(
                        "Y軸範囲", 
                        min_value=0.0, 
                        max_value=float(max(values) * 1.1),
                        value=(y_min_default, y_max_default),
                        step=0.01
                    )
        else:
            clip_outliers = True
            y_range = None
        
        # 最適化履歴
        fig_history = plot_optimization_history(study, clip_outliers=clip_outliers)
        
        # 手動Y軸範囲を適用
        if y_range is not None:
            fig_history.update_layout(yaxis=dict(range=[y_range[0], y_range[1]]))
        
        st.plotly_chart(fig_history, use_container_width=True)
        
        st.caption("💡 ヒント: グラフ上でドラッグして範囲選択でもズームできます（ダブルクリックでリセット）")
        
        # シナリオ別スコア
        if trial_history:
            fig_scenarios = plot_scenario_scores(trial_history)
            if fig_scenarios:
                st.plotly_chart(fig_scenarios, use_container_width=True)
    
    with tab2:
        # 軌跡比較タブ
        st.subheader("🗺️ 軌跡比較 (実機 vs 初期 vs ベスト)")
        
        # 軌跡データを読み込み
        trajectories = None
        try:
            with open(results_path, 'r') as f:
                data = json.load(f)
                trajectories = data.get("trajectories", {})
        except Exception as e:
            st.warning(f"軌跡データの読み込みに失敗しました: {e}")
        
        if trajectories:
            initial_traj = trajectories.get("initial", {})
            best_traj = trajectories.get("best", {})
            
            # デバッグ情報
            st.caption(f"📁 ファイル: {results_path}")
            st.caption(f"初期軌跡: {list(initial_traj.keys())} | ベスト軌跡: {list(best_traj.keys())}")
            
            all_scenarios = set(initial_traj.keys()) | set(best_traj.keys())
            
            if len(all_scenarios) == 0:
                st.info("まだ軌跡データがありません。最適化を実行してください。")
            else:
                # 2列レイアウト
                cols = st.columns(2)
                
                for idx, scenario_name in enumerate(sorted(all_scenarios)):
                    with cols[idx % 2]:
                        st.markdown(f"**{scenario_name}**")
                        
                        fig = go.Figure()
                        
                        # 実機軌跡 (どちらかのデータから取得)
                        if scenario_name in best_traj and "real_x" in best_traj[scenario_name]:
                            real_data = best_traj[scenario_name]
                            fig.add_trace(go.Scatter(
                                x=real_data["real_x"],
                                y=real_data["real_y"],
                                mode='lines',
                                name='実機',
                                line=dict(color='blue', width=2)
                            ))
                        elif scenario_name in initial_traj and "real_x" in initial_traj[scenario_name]:
                            real_data = initial_traj[scenario_name]
                            fig.add_trace(go.Scatter(
                                x=real_data["real_x"],
                                y=real_data["real_y"],
                                mode='lines',
                                name='実機',
                                line=dict(color='blue', width=2)
                            ))
                        
                        # 初期パラメータ軌跡
                        if scenario_name in initial_traj:
                            init_data = initial_traj[scenario_name]
                            rmse = init_data.get("rmse", 0)
                            
                            # Use raw simulation data if available
                            if "sim_raw_x" in init_data and len(init_data["sim_raw_x"]) > 0:
                                sx, sy = init_data["sim_raw_x"], init_data["sim_raw_y"]
                                lbl = f'初期 (RMSE={rmse:.3f}m)'
                            else:
                                sx, sy = init_data["sim_x"], init_data["sim_y"]
                                lbl = f'初期 (Interp, RMSE={rmse:.3f}m)'

                            fig.add_trace(go.Scatter(
                                x=sx,
                                y=sy,
                                mode='lines',
                                name=lbl,
                                line=dict(color='orange', width=3, dash='dot')
                            ))
                        
                        # ベストパラメータ軌跡
                        if scenario_name in best_traj:
                            best_data = best_traj[scenario_name]
                            rmse = best_data.get("rmse", 0)
                            
                            # Use raw simulation data if available
                            if "sim_raw_x" in best_data and len(best_data["sim_raw_x"]) > 0:
                                sx, sy = best_data["sim_raw_x"], best_data["sim_raw_y"]
                                lbl = f'ベスト (RMSE={rmse:.3f}m)'
                            else:
                                sx, sy = best_data["sim_x"], best_data["sim_y"]
                                lbl = f'ベスト (Interp, RMSE={rmse:.3f}m)'
                                
                            fig.add_trace(go.Scatter(
                                x=sx,
                                y=sy,
                                mode='lines',
                                name=lbl,
                                line=dict(color='red', width=3)
                            ))
                        
                        fig.update_layout(
                            xaxis_title="X [m]",
                            yaxis_title="Y [m]",
                            height=450,
                            legend=dict(
                                yanchor="top", y=0.99, 
                                xanchor="left", x=0.01,
                                bgcolor="rgba(255,255,255,0.8)"
                            ),
                            margin=dict(l=40, r=20, t=30, b=40)
                        )
                        fig.update_yaxes(scaleanchor="x", scaleratio=1)
                        
                        st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("軌跡データがまだありません。最適化を実行してください。")
    
    with tab3:
        col1, col2 = st.columns(2)
        
        with col1:
            # パラメータ重要度
            fig_importance = plot_param_importance(study)
            if fig_importance:
                st.plotly_chart(fig_importance, use_container_width=True)
            else:
                st.info("パラメータ重要度の計算にはより多くの試行が必要です")
        
        with col2:
            # スコア分布
            values = [t.value for t in study.trials if t.value is not None]
            if values:
                fig_dist = go.Figure(data=[go.Histogram(x=values, nbinsx=30)])
                fig_dist.update_layout(
                    title="Score Distribution",
                    xaxis_title="Score",
                    yaxis_title="Frequency"
                )
                st.plotly_chart(fig_dist, use_container_width=True)
        
        # パラレル座標
        fig_parallel = plot_parallel_coordinates(study)
        if fig_parallel:
            st.plotly_chart(fig_parallel, use_container_width=True)
    
    with tab4:
        display_best_params(study)
        
        # パラメータをコピー可能な形式で表示
        if study.best_params:
            st.subheader("📋 Copy-Paste Format")
            params_json = json.dumps(study.best_params, indent=2)
            st.code(params_json, language='json')
        
        # Scenario-wise Breakdown for Best Trial
        st.divider()
        st.subheader("📊 Best Score Breakdown")
        
        if trial_history and study.best_trial:
            best_trial_num = study.best_trial.number
            best_trial_data = next((t for t in trial_history if t['trial_number'] == best_trial_num), None)
            
            if best_trial_data and 'scenario_scores' in best_trial_data:
                scores = best_trial_data['scenario_scores']
                
                # Bar Chart
                fig_breakdown = go.Figure(go.Bar(
                    x=list(scores.values()),
                    y=list(scores.keys()),
                    orientation='h',
                    marker_color='salmon',
                    text=[f"{v:.4f}" for v in scores.values()],
                    textposition='auto'
                ))
                
                fig_breakdown.update_layout(
                    title=f"RMSE per Scenario (Total: {study.best_value:.4f})",
                    xaxis_title="RMSE [m]",
                    yaxis=dict(autorange="reversed"),
                    height=300,
                    margin=dict(l=20, r=20, t=40, b=20)
                )
                
                col_chart, col_table = st.columns([2, 1])
                
                with col_chart:
                    st.plotly_chart(fig_breakdown, use_container_width=True)
                
                with col_table:
                    st.write("Results Table")
                    df_scores = pd.DataFrame(list(scores.items()), columns=['Scenario', 'RMSE'])
                    # st.dataframe(df_scores, hide_index=True) # hide_index is new in recent streamlit
                    st.table(df_scores.set_index('Scenario'))
            else:
                st.info("No breakdown data available for the best trial.")
    
    with tab5:
        # 試行一覧テーブル
        if len(study.trials) > 0:
            data = []
            for t in study.trials:
                row = {
                    "Trial": t.number,
                    "Score": t.value if t.value is not None else None,
                    "State": t.state.name
                }
                # 主要パラメータを追加
                for key in ["wheel_kp", "skidding_cancel_rate"]:
                    if key in t.params:
                        row[key] = f"{t.params[key]:.3f}"
                data.append(row)
            
            df = pd.DataFrame(data)
            st.dataframe(df, use_container_width=True)
    
    # 自動リフレッシュ
    if auto_refresh:
        time.sleep(refresh_interval)
        st.rerun()


if __name__ == "__main__":
    main()
