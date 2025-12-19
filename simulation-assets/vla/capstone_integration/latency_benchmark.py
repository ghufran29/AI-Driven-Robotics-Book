"""
Latency Benchmarking Tool for VLA System

This module measures and benchmarks the latency of the VLA pipeline
to ensure it meets the <5 second requirement from voice command to robot start.
"""
import asyncio
import time
import statistics
from typing import Dict, Any, List, Optional
import logging
import json
from datetime import datetime
import csv

logger = logging.getLogger(__name__)

class LatencyBenchmark:
    """
    Class for measuring and benchmarking VLA pipeline latency.
    """
    def __init__(self, target_latency: float = 5.0, warmup_runs: int = 3):
        """
        Initialize the latency benchmark tool.

        Args:
            target_latency: Target latency in seconds (default: 5.0)
            warmup_runs: Number of warmup runs before benchmarking
        """
        self.target_latency = target_latency
        self.warmup_runs = warmup_runs
        self.benchmark_results = []
        self.stage_latencies = {
            'voice_to_text': [],
            'language_understanding': [],
            'vision_processing': [],
            'action_validation': [],
            'total': []
        }

    async def benchmark_pipeline(self, pipeline: 'VLAPipeline', test_commands: List[str],
                                num_runs: int = 10) -> Dict[str, Any]:
        """
        Benchmark the VLA pipeline with test commands.

        Args:
            pipeline: VLAPipeline instance to benchmark
            test_commands: List of test commands to process
            num_runs: Number of runs for each command

        Returns:
            Dictionary with benchmark results
        """
        logger.info(f"Starting latency benchmark with {num_runs} runs per command...")

        # Warmup runs
        logger.info(f"Performing {self.warmup_runs} warmup runs...")
        for i in range(self.warmup_runs):
            # Use a simple command for warmup
            warmup_result = await pipeline.process_command_string("Move to position x=1.0 y=1.0")
            if warmup_result:
                logger.debug(f"Warmup run {i+1} completed")

        # Reset results
        self.benchmark_results = []
        self.stage_latencies = {key: [] for key in self.stage_latencies.keys()}

        # Run benchmarks
        total_start_time = time.time()

        for run in range(num_runs):
            for command in test_commands:
                run_start_time = time.time()

                try:
                    result = await pipeline.process_command_string(command)

                    if result and result["success"]:
                        run_time = time.time() - run_start_time

                        # Store the result
                        benchmark_record = {
                            "run": run,
                            "command": command,
                            "success": True,
                            "total_latency": result["latency"]["total"],
                            "timestamp": result["timestamp"],
                            "detailed_latency": result["latency"]
                        }

                        self.benchmark_results.append(benchmark_record)

                        # Store stage latencies
                        if "detailed_latency" in result:
                            for stage, latency in result["latency"].items():
                                if stage in self.stage_latencies:
                                    self.stage_latencies[stage].append(latency)

                        logger.debug(f"Run {run+1}, Command '{command}': {run_time:.3f}s")

                    else:
                        # Record failure
                        benchmark_record = {
                            "run": run,
                            "command": command,
                            "success": False,
                            "error": result.get("error", "Unknown error") if result else "No result",
                            "timestamp": time.time()
                        }
                        self.benchmark_results.append(benchmark_record)

                except Exception as e:
                    logger.error(f"Error during benchmark run {run+1}: {str(e)}")
                    benchmark_record = {
                        "run": run,
                        "command": command,
                        "success": False,
                        "error": str(e),
                        "timestamp": time.time()
                    }
                    self.benchmark_results.append(benchmark_record)

        total_time = time.time() - total_start_time

        # Calculate statistics
        results = self._calculate_statistics()
        results['total_benchmark_time'] = total_time
        results['total_runs'] = len(self.benchmark_results)
        results['successful_runs'] = len([r for r in self.benchmark_results if r['success']])
        results['failure_rate'] = 1 - (results['successful_runs'] / results['total_runs']) if results['total_runs'] > 0 else 1.0

        # Add target compliance information
        results['target_latency'] = self.target_latency
        results['meets_target'] = results['mean_latency'] <= self.target_latency
        results['target_compliance_rate'] = results['total_runs_within_target'] / results['total_runs'] if results['total_runs'] > 0 else 0.0

        logger.info(f"Benchmark completed. Total time: {total_time:.2f}s, Success rate: {results['success_rate']:.2%}")

        return results

    def _calculate_statistics(self) -> Dict[str, Any]:
        """
        Calculate statistics from benchmark results.

        Returns:
            Dictionary with calculated statistics
        """
        successful_results = [r for r in self.benchmark_results if r['success']]

        if not successful_results:
            return {
                "mean_latency": 0.0,
                "median_latency": 0.0,
                "std_deviation": 0.0,
                "min_latency": 0.0,
                "max_latency": 0.0,
                "p95_latency": 0.0,
                "p99_latency": 0.0,
                "total_runs_within_target": 0,
                "success_rate": 0.0,
                "stage_statistics": {}
            }

        latencies = [r['total_latency'] for r in successful_results]

        # Calculate basic statistics
        mean_latency = statistics.mean(latencies)
        median_latency = statistics.median(latencies)
        min_latency = min(latencies)
        max_latency = max(latencies)

        std_deviation = 0.0
        if len(latencies) > 1:
            std_deviation = statistics.stdev(latencies)

        # Calculate percentiles
        sorted_latencies = sorted(latencies)
        p95_index = int(0.95 * len(sorted_latencies))
        p99_index = int(0.99 * len(sorted_latencies))

        p95_latency = sorted_latencies[min(p95_index, len(sorted_latencies) - 1)]
        p99_latency = sorted_latencies[min(p99_index, len(sorted_latencies) - 1)]

        # Count runs within target latency
        runs_within_target = sum(1 for lat in latencies if lat <= self.target_latency)

        # Calculate stage statistics
        stage_stats = {}
        for stage, stage_latencies in self.stage_latencies.items():
            if stage_latencies:
                stage_stats[stage] = {
                    "mean": statistics.mean(stage_latencies),
                    "median": statistics.median(stage_latencies),
                    "min": min(stage_latencies),
                    "max": max(stage_latencies),
                    "std_dev": statistics.stdev(stage_latencies) if len(stage_latencies) > 1 else 0.0,
                    "count": len(stage_latencies)
                }

        return {
            "mean_latency": mean_latency,
            "median_latency": median_latency,
            "std_deviation": std_deviation,
            "min_latency": min_latency,
            "max_latency": max_latency,
            "p95_latency": p95_latency,
            "p99_latency": p99_latency,
            "total_runs_within_target": runs_within_target,
            "success_rate": len(successful_results) / len(self.benchmark_results) if self.benchmark_results else 0.0,
            "stage_statistics": stage_stats
        }

    def generate_report(self, results: Dict[str, Any]) -> str:
        """
        Generate a human-readable benchmark report.

        Args:
            results: Results dictionary from benchmark_pipeline

        Returns:
            Formatted report string
        """
        report = []
        report.append("=" * 60)
        report.append("VLA Pipeline Latency Benchmark Report")
        report.append("=" * 60)
        report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"Target Latency: {results['target_latency']}s")
        report.append("")

        # Overall results
        report.append("Overall Performance:")
        report.append(f"  Total Runs: {results['total_runs']}")
        report.append(f"  Successful Runs: {results['successful_runs']}")
        report.append(f"  Success Rate: {results['success_rate']:.2%}")
        report.append(f"  Failure Rate: {results['failure_rate']:.2%}")
        report.append(f"  Meets Target: {'YES' if results['meets_target'] else 'NO'}")
        report.append(f"  Target Compliance: {results['target_compliance_rate']:.2%}")
        report.append("")

        # Latency statistics
        report.append("Latency Statistics:")
        report.append(f"  Mean Latency: {results['mean_latency']:.3f}s")
        report.append(f"  Median Latency: {results['median_latency']:.3f}s")
        report.append(f"  Std Deviation: {results['std_deviation']:.3f}s")
        report.append(f"  Min Latency: {results['min_latency']:.3f}s")
        report.append(f"  Max Latency: {results['max_latency']:.3f}s")
        report.append(f"  P95 Latency: {results['p95_latency']:.3f}s")
        report.append(f"  P99 Latency: {results['p99_latency']:.3f}s")
        report.append("")

        # Stage statistics
        report.append("Stage Performance:")
        for stage, stats in results['stage_statistics'].items():
            report.append(f"  {stage}:")
            report.append(f"    Mean: {stats['mean']:.3f}s")
            report.append(f"    Min: {stats['min']:.3f}s")
            report.append(f"    Max: {stats['max']:.3f}s")
            report.append(f"    Std Dev: {stats['std_dev']:.3f}s")
            report.append(f"    Count: {stats['count']}")
        report.append("")

        # Compliance summary
        report.append("Compliance Summary:")
        report.append(f"  Runs within {results['target_latency']}s: {results['total_runs_within_target']}/{results['total_runs']}")
        report.append(f"  Compliance Rate: {results['target_compliance_rate']:.2%}")
        report.append("=" * 60)

        return "\n".join(report)

    def save_results(self, results: Dict[str, Any], filepath: str):
        """
        Save benchmark results to a file.

        Args:
            results: Results dictionary from benchmark_pipeline
            filepath: Path to save the results
        """
        # Save detailed results as JSON
        json_path = filepath.replace('.csv', '.json')
        with open(json_path, 'w') as f:
            json.dump({
                "benchmark_info": {
                    "timestamp": datetime.now().isoformat(),
                    "target_latency": self.target_latency,
                    "total_runs": results['total_runs'],
                    "successful_runs": results['successful_runs']
                },
                "statistics": {
                    "overall": {
                        "mean_latency": results['mean_latency'],
                        "median_latency": results['median_latency'],
                        "std_deviation": results['std_deviation'],
                        "min_latency": results['min_latency'],
                        "max_latency": results['max_latency'],
                        "p95_latency": results['p95_latency'],
                        "p99_latency": results['p99_latency'],
                        "success_rate": results['success_rate'],
                        "target_compliance_rate": results['target_compliance_rate']
                    },
                    "stages": results['stage_statistics']
                },
                "raw_results": self.benchmark_results
            }, f, indent=2)

        # Save summary as CSV
        csv_path = filepath
        with open(csv_path, 'w', newline='') as csvfile:
            fieldnames = [
                'run', 'command', 'success', 'total_latency', 'voice_to_text',
                'language_understanding', 'vision_processing', 'action_validation'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for result in self.benchmark_results:
                row = {
                    'run': result.get('run', ''),
                    'command': result.get('command', ''),
                    'success': result.get('success', ''),
                    'total_latency': result.get('total_latency', ''),
                    'voice_to_text': '',
                    'language_understanding': '',
                    'vision_processing': '',
                    'action_validation': ''
                }

                # Add detailed latency if available
                if result.get('detailed_latency'):
                    for stage in ['voice_to_text', 'language_understanding', 'vision_processing', 'action_validation']:
                        if stage in result['detailed_latency']:
                            row[stage] = result['detailed_latency'][stage]

                writer.writerow(row)

        logger.info(f"Benchmark results saved to {json_path} and {csv_path}")

    def get_performance_recommendations(self, results: Dict[str, Any]) -> List[str]:
        """
        Generate performance recommendations based on benchmark results.

        Args:
            results: Results dictionary from benchmark_pipeline

        Returns:
            List of recommendations
        """
        recommendations = []

        # Check overall latency
        if results['mean_latency'] > self.target_latency:
            recommendations.append(f"Overall latency ({results['mean_latency']:.3f}s) exceeds target ({self.target_latency}s). Consider optimization.")

        # Check stage performance
        stage_stats = results.get('stage_statistics', {})
        for stage, stats in stage_stats.items():
            if stats['mean'] > 1.0:  # If any stage takes more than 1 second on average
                recommendations.append(f"Stage '{stage}' has high latency ({stats['mean']:.3f}s). Consider optimization.")

        # Check success rate
        if results['success_rate'] < 0.95:  # Less than 95% success rate
            recommendations.append(f"Low success rate ({results['success_rate']:.2%}). Investigate failures.")

        # Check compliance rate
        if results['target_compliance_rate'] < 0.95:  # Less than 95% compliance
            recommendations.append(f"Low target compliance ({results['target_compliance_rate']:.2%}). Optimize for consistency.")

        if not recommendations:
            recommendations.append("Performance looks good! All metrics are within acceptable ranges.")

        return recommendations


async def run_sample_benchmark():
    """
    Run a sample benchmark with a mock pipeline.
    In a real scenario, this would use an actual VLAPipeline instance.
    """
    print("Running sample latency benchmark...")

    # Since we don't have a real pipeline during this phase,
    # we'll simulate the benchmark with mock data
    benchmark = LatencyBenchmark(target_latency=5.0, warmup_runs=2)

    # Generate mock results to simulate pipeline performance
    import random
    mock_results = []

    for run in range(10):  # 10 mock runs
        for command in ["Move to position x=1.0 y=1.0", "Pick up object", "Navigate to kitchen"]:
            success = random.random() > 0.1  # 90% success rate in mock
            if success:
                # Generate realistic latency values
                total_latency = random.uniform(1.5, 4.0)  # Between 1.5 and 4 seconds

                mock_record = {
                    "run": run,
                    "command": command,
                    "success": True,
                    "total_latency": total_latency,
                    "timestamp": time.time(),
                    "detailed_latency": {
                        "voice_to_text": random.uniform(0.2, 0.8),
                        "language_understanding": random.uniform(0.5, 1.5),
                        "vision_processing": random.uniform(0.3, 0.9),
                        "action_validation": random.uniform(0.1, 0.3)
                    }
                }
            else:
                mock_record = {
                    "run": run,
                    "command": command,
                    "success": False,
                    "error": "Simulated error",
                    "timestamp": time.time()
                }

            mock_results.append(mock_record)

    # Store mock results in benchmark object
    benchmark.benchmark_results = mock_results

    # Calculate mock stage latencies
    for result in mock_results:
        if result['success'] and 'detailed_latency' in result:
            for stage, latency in result['detailed_latency'].items():
                if stage in benchmark.stage_latencies:
                    benchmark.stage_latencies[stage].append(latency)

    # Calculate statistics from mock results
    results = benchmark._calculate_statistics()
    results['target_latency'] = benchmark.target_latency
    results['meets_target'] = results['mean_latency'] <= benchmark.target_latency
    results['total_runs'] = len(mock_results)
    results['successful_runs'] = len([r for r in mock_results if r['success']])
    results['failure_rate'] = 1 - (results['successful_runs'] / results['total_runs']) if results['total_runs'] > 0 else 1.0
    results['total_runs_within_target'] = sum(1 for r in mock_results if r['success'] and r['total_latency'] <= benchmark.target_latency)

    # Generate and print report
    report = benchmark.generate_report(results)
    print(report)

    # Get recommendations
    recommendations = benchmark.get_performance_recommendations(results)
    print("\nPerformance Recommendations:")
    for rec in recommendations:
        print(f"  - {rec}")

    # Save mock results
    benchmark.save_results(results, "mock_benchmark_results.csv")

    print(f"\n✓ Sample benchmark completed")
    print(f"  - Success rate: {results['success_rate']:.2%}")
    print(f"  - Mean latency: {results['mean_latency']:.3f}s")
    print(f"  - Target compliance: {results['target_compliance_rate']:.2%}")

    return True


async def main():
    """Main function to run the benchmark."""
    success = await run_sample_benchmark()
    if success:
        print("\n✓ Latency benchmark tool test completed successfully")
    else:
        print("\n✗ Latency benchmark tool test failed")


if __name__ == "__main__":
    asyncio.run(main())