"""
メインエントリーポイント

最適化の実行、可視化、ダッシュボード起動を制御します。

使用方法:
    # 最適化実行
    python main.py optimize --trials 100
    
    # 可視化のみ
    python main.py visualize
    
    # ダッシュボード起動
    python main.py dashboard
"""

import argparse
import sys
from pathlib import Path

from .paths import DEFAULT_ROSBAG_DIR, DEFAULT_SEARCH_SPACE


def cmd_optimize(args):
    """最適化を実行"""
    from .optimizer import OptimizerConfig, run_optimization
    from .visualizer import create_visualization_report
    
    config = OptimizerConfig(
        n_trials=args.trials,
        timeout=args.timeout,
        scenarios=args.scenarios.split(',') if args.scenarios else None,
        time_scale=args.time_scale,
        grpc_host=args.host,
        grpc_port=args.port,
        rosbag_base_path=args.rosbag_path,
        study_name=args.study_name,
        storage_path=f"sqlite:///{args.db_path}",
        results_json_path=args.results_json,
        search_space_path=args.search_space,
    )
    
    print("=" * 60)
    print("AWSIM Parameter Optimization")
    print("=" * 60)
    print(f"Trials: {config.n_trials}")
    print(f"Search space: {config.search_space_path}")
    print(f"Scenarios: {config.scenarios or 'all'}")
    print(f"Database: {args.db_path}")
    print(f"Results JSON: {config.results_json_path}")
    print(f"TimeScale: {config.time_scale}x")
    print(f"gRPC: {config.grpc_host}:{config.grpc_port}")
    print("=" * 60)
    
    try:
        study = run_optimization(config)
        
        # 可視化生成
        print("\nGenerating visualization...")
        create_visualization_report(study, args.output_dir)
        
        print("\n✅ Optimization complete!")
        print(f"Best score: {study.best_value:.4f}")
        print(f"Results saved to: {args.output_dir}/")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)


def cmd_visualize(args):
    """可視化を生成"""
    import optuna
    from .visualizer import create_visualization_report
    
    try:
        study = optuna.load_study(
            study_name=args.study_name,
            storage=f"sqlite:///{args.db_path}"
        )
        
        create_visualization_report(study, args.output_dir)
        print(f"✅ Visualization saved to: {args.output_dir}/")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


def cmd_dashboard(args):
    """ダッシュボードを起動"""
    import subprocess
    
    dashboard_path = Path(__file__).parent / "dashboard.py"
    
    cmd = [
        "streamlit", "run", str(dashboard_path),
        "--",
        "--study-name", args.study_name,
        "--db-path", f"sqlite:///{args.db_path}",
        "--results-json", args.results_json,
    ]
    
    print("Starting dashboard...")
    print(f"Open: http://localhost:8501")
    print(f"DB: {args.db_path} | JSON: {args.results_json}")
    
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nDashboard stopped.")


def cmd_test_rosbag(args):
    """rosbag読み込みテスト"""
    from .rosbag_loader import load_all_scenarios
    
    print(f"Loading scenarios from: {args.rosbag_path}")
    scenario_filter = args.scenarios.split(',') if args.scenarios else None
    if scenario_filter:
        print(f"Filter: {scenario_filter}")
        
    scenarios = load_all_scenarios(args.rosbag_path, scenario_filter)
    
    if len(scenarios) == 0:
        print("❌ No scenarios found!")
        sys.exit(1)
    
    for name, data in scenarios.items():
        print(f"\n=== {name} ===")
        print(f"  Duration: {data.duration:.2f}s")
        print(f"  Odom samples: {len(data.odom.timestamps)}")
        print(f"  CmdVel samples: {len(data.cmd_vel.timestamps)}")
        print(f"  X range: [{data.odom.positions_x.min():.3f}, {data.odom.positions_x.max():.3f}]")
        print(f"  Y range: [{data.odom.positions_y.min():.3f}, {data.odom.positions_y.max():.3f}]")
    
    print(f"\n✅ Successfully loaded {len(scenarios)} scenarios")


def cmd_test_grpc(args):
    """gRPC接続テスト"""
    from .grpc_client import GrpcClient
    
    client = GrpcClient(args.host, args.port)
    
    print(f"Connecting to {args.host}:{args.port}...")
    
    if client.connect():
        print("✅ Connected to AWSIM!")
        
        # Odom取得テスト
        odom = client.get_current_odom()
        if odom:
            print(f"  Current position: ({odom.position[0]:.3f}, {odom.position[1]:.3f}, {odom.position[2]:.3f})")
        
        client.disconnect()
    else:
        print("❌ Failed to connect. Make sure AWSIM is running with the OptimizerBridge.")
        sys.exit(1)


def cmd_ttest(args):
    """t検定によるパラメータ検証を実行"""
    from .ttest import run_ttest
    run_ttest()


def cmd_ttest3(args):
    """3グループ比較t検定を実行"""
    from .ttest import run_ttest_triple
    run_ttest_triple()


def cmd_batch_eval(args):
    """パラメータ一括評価を実行"""
    from .batch_eval import run_batch_eval
    run_batch_eval(
        param_dir=args.param_dir,
        rosbag_path=args.rosbag_path,
        grpc_host=args.host,
        grpc_port=args.port,
        output_prefix=args.output_prefix
    )


def cmd_plot_trajectory(args):
    """軌跡プロットを編集"""
    from .plot_trajectory import run_plot_trajectory
    # 引数をパースして渡す
    cmd_args = [args.json_file]
    if args.output:
        cmd_args.extend(['-o', args.output])
    if args.xlim:
        cmd_args.extend(['--xlim', args.xlim])
    if args.ylim:
        cmd_args.extend(['--ylim', args.ylim])
    if args.interactive:
        cmd_args.append('-i')
    run_plot_trajectory(cmd_args)


def _load_vehicle_params_yaml(path):
    import yaml
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('vehicle_params', data)


def cmd_apply_params(args):
    """YAML の車両パラメータを実行中の AWSIM に反映する"""
    from .grpc_client import GrpcClient, VehicleParameters

    values = _load_vehicle_params_yaml(args.yaml)
    params = VehicleParameters()
    unknown = []
    for key, value in values.items():
        if hasattr(params, key):
            setattr(params, key, float(value))
        else:
            unknown.append(key)
    if unknown:
        print(f"Ignored (not settable at runtime): {', '.join(unknown)}")

    client = GrpcClient(args.host, args.port)
    if not client.connect():
        print("❌ Failed to connect. Make sure AWSIM is playing and OptimizerBridge is enabled.")
        sys.exit(1)
    try:
        if not client.set_parameters(params):
            print("❌ set_parameters failed")
            sys.exit(1)
        print(f"✅ Applied {args.yaml}")
        current = client.get_parameters()
        if current is not None:
            for key in sorted(current):
                print(f"  {key}: {current[key]}")
    finally:
        client.disconnect()


def cmd_get_params(args):
    """実行中の AWSIM の車両パラメータを表示 (--output で YAML に保存)"""
    from .grpc_client import GrpcClient

    client = GrpcClient(args.host, args.port)
    if not client.connect():
        print("❌ Failed to connect. Make sure AWSIM is playing and OptimizerBridge is enabled.")
        sys.exit(1)
    try:
        current = client.get_parameters()
    finally:
        client.disconnect()
    if current is None:
        print("❌ get_parameters failed")
        sys.exit(1)
    for key in sorted(current):
        print(f"{key}: {current[key]}")
    if args.output:
        import yaml
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, 'w') as f:
            yaml.safe_dump({'vehicle_params': current}, f, sort_keys=False)
        print(f"Saved to {args.output}")


def cmd_diff_params(args):
    """2 つのパラメータ YAML を並べて比較する"""
    a = _load_vehicle_params_yaml(args.yaml_a)
    b = _load_vehicle_params_yaml(args.yaml_b)
    keys = list(dict.fromkeys(list(a) + list(b)))
    width = max(len(k) for k in keys)
    print(f"{'parameter':<{width}}  {Path(args.yaml_a).name:>22}  {Path(args.yaml_b).name:>22}  diff")
    for k in keys:
        va, vb = a.get(k), b.get(k)
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
            mark = "" if abs(float(va) - float(vb)) <= args.tolerance * max(1.0, abs(float(va))) else "*"
            print(f"{k:<{width}}  {float(va):>22.6g}  {float(vb):>22.6g}  {mark}")
        else:
            print(f"{k:<{width}}  {str(va):>22}  {str(vb):>22}  *")


def main():
    parser = argparse.ArgumentParser(
        description="AWSIM Vehicle Parameter Optimizer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run optimization with 100 trials
  python3 -m whill_optimizer optimize --trials 100
  
  # Run optimization on specific scenarios
  python3 -m whill_optimizer optimize --trials 50 --scenarios straight,turn
  
  # Generate visualization from existing study
  python3 -m whill_optimizer visualize
  
  # Start dashboard
  python3 -m whill_optimizer dashboard
  
  # Test rosbag loading
  python3 -m whill_optimizer test-rosbag
  
  # Test gRPC connection
  python3 -m whill_optimizer test-grpc
  
  # Run t-test (2 groups)
  python3 -m whill_optimizer ttest
  
  # Run t-test (3 groups comparison)
  python3 -m whill_optimizer ttest3
  
  # Batch evaluation of all parameters
  python3 -m whill_optimizer batch-eval --param-dir ../parameter
        """
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # optimize コマンド
    p_opt = subparsers.add_parser("optimize", help="Run optimization")
    p_opt.add_argument("--trials", type=int, default=100, help="Number of trials")
    p_opt.add_argument("--timeout", type=float, default=None, help="Timeout in seconds")
    p_opt.add_argument("--scenarios", type=str, default=None, 
                       help="Comma-separated scenario names (default: all)")
    p_opt.add_argument("--time-scale", type=float, default=1.0, help="Simulation time scale")
    p_opt.add_argument("--host", type=str, default="localhost", help="gRPC host")
    p_opt.add_argument("--port", type=int, default=50051, help="gRPC port")
    p_opt.add_argument("--rosbag-path", type=str, default=DEFAULT_ROSBAG_DIR,
                       help="Base path for rosbag2 files")
    p_opt.add_argument("--study-name", type=str, default="awsim_optimization", help="Optuna study name")
    p_opt.add_argument("--db-path", type=str, default="optuna_study.db", help="SQLite database path")
    p_opt.add_argument("--results-json", type=str, default="optimization_results.json", 
                       help="Results JSON file path")
    p_opt.add_argument("--output-dir", type=str, default="./visualization", help="Output directory")
    p_opt.add_argument("--search-space", type=str, default=DEFAULT_SEARCH_SPACE,
                       help="Search space YAML (default: config/optimizer/search_space.yaml)")
    p_opt.set_defaults(func=cmd_optimize)
    
    # visualize コマンド
    p_viz = subparsers.add_parser("visualize", help="Generate visualization")
    p_viz.add_argument("--study-name", type=str, default="awsim_optimization", help="Optuna study name")
    p_viz.add_argument("--db-path", type=str, default="optuna_study.db", help="SQLite database path")
    p_viz.add_argument("--output-dir", type=str, default="./visualization", help="Output directory")
    p_viz.set_defaults(func=cmd_visualize)
    
    # dashboard コマンド
    p_dash = subparsers.add_parser("dashboard", help="Start Streamlit dashboard")
    p_dash.add_argument("--study-name", type=str, default="awsim_optimization", help="Optuna study name")
    p_dash.add_argument("--db-path", type=str, default="optuna_study.db", help="SQLite database path")
    p_dash.add_argument("--results-json", type=str, default="optimization_results.json", help="Results JSON file")
    p_dash.set_defaults(func=cmd_dashboard)
    
    # test-rosbag コマンド
    p_test_bag = subparsers.add_parser("test-rosbag", help="Test rosbag loading")
    p_test_bag.add_argument("--rosbag-path", type=str, default=DEFAULT_ROSBAG_DIR,
                            help="Base path for rosbag2 files")
    p_test_bag.add_argument("--scenarios", type=str, default=None, 
                           help="Comma-separated scenario names (default: all)")
    p_test_bag.set_defaults(func=cmd_test_rosbag)
    
    # test-grpc コマンド
    p_test_grpc = subparsers.add_parser("test-grpc", help="Test gRPC connection")
    p_test_grpc.add_argument("--host", type=str, default="localhost", help="gRPC host")
    p_test_grpc.add_argument("--port", type=int, default=50051, help="gRPC port")
    p_test_grpc.set_defaults(func=cmd_test_grpc)
    
    # ttest コマンド
    p_ttest = subparsers.add_parser("ttest", help="Run t-test parameter validation (2 groups)")
    p_ttest.set_defaults(func=cmd_ttest)
    
    # ttest3 コマンド
    p_ttest3 = subparsers.add_parser("ttest3", help="Run t-test parameter validation (3 groups)")
    p_ttest3.set_defaults(func=cmd_ttest3)
    
    # plot-trajectory コマンド
    p_plot = subparsers.add_parser("plot-trajectory", help="Edit trajectory plot from JSON")
    p_plot.add_argument("json_file", help="軌跡データJSON (ttest_trajectory_*.json)")
    p_plot.add_argument("-o", "--output", help="出力ファイル (pdf/png)")
    p_plot.add_argument("--xlim", help="X軸範囲 (min,max)")
    p_plot.add_argument("--ylim", help="Y軸範囲 (min,max)")
    p_plot.add_argument("-i", "--interactive", action="store_true", help="対話モード")
    p_plot.set_defaults(func=cmd_plot_trajectory)
    
    # batch-eval コマンド
    p_batch = subparsers.add_parser("batch-eval", help="Batch evaluation of all parameters")
    p_batch.add_argument("--param-dir", type=str, default="../parameter",
                         help="Directory containing parameter YAML files")
    p_batch.add_argument("--rosbag-path", type=str, default=DEFAULT_ROSBAG_DIR,
                         help="Base path for rosbag2 files")
    p_batch.add_argument("--host", type=str, default="localhost", help="gRPC host")
    p_batch.add_argument("--port", type=int, default=50051, help="gRPC port")
    p_batch.add_argument("--output-prefix", type=str, default="batch_eval",
                         help="Output file prefix")
    p_batch.set_defaults(func=cmd_batch_eval)
    
    
    # apply-params コマンド
    p_apply = subparsers.add_parser("apply-params", help="Apply a vehicle parameter YAML to the running AWSIM")
    p_apply.add_argument("--yaml", type=str, required=True, help="Parameter YAML (vehicle_params: ...)")
    p_apply.add_argument("--host", type=str, default="localhost", help="OptimizerBridge host")
    p_apply.add_argument("--port", type=int, default=50051, help="OptimizerBridge port")
    p_apply.set_defaults(func=cmd_apply_params)

    # get-params コマンド
    p_get = subparsers.add_parser("get-params", help="Show the vehicle parameters of the running AWSIM")
    p_get.add_argument("--host", type=str, default="localhost", help="OptimizerBridge host")
    p_get.add_argument("--port", type=int, default=50051, help="OptimizerBridge port")
    p_get.add_argument("--output", type=str, default=None, help="Save as YAML")
    p_get.set_defaults(func=cmd_get_params)

    # diff-params コマンド
    p_diff = subparsers.add_parser("diff-params", help="Compare two parameter YAML files")
    p_diff.add_argument("yaml_a", type=str)
    p_diff.add_argument("yaml_b", type=str)
    p_diff.add_argument("--tolerance", type=float, default=1e-6, help="Relative tolerance for '*' marks")
    p_diff.set_defaults(func=cmd_diff_params)

    args = parser.parse_args()
    
    if args.command is None:
        parser.print_help()
        sys.exit(0)
    
    args.func(args)


if __name__ == "__main__":
    main()
