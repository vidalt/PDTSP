#/bin/python3

import json
import sys
import argparse

if __name__ == "__main__":
    def parse_arguments(argv):
        parser = argparse.ArgumentParser(
            description="Compare two outputs from pdtsp solvers.",
            epilog="Compare two outputs from pdtsp solvers.",
        )
        parser.add_argument("--current", dest="res_current", required=True, help="current version output", type=str)
        parser.add_argument("--expected", dest="res_expected", required=True, help="expected version output", type=str)
        return parser.parse_args(argv[1:])

    args = parse_arguments(sys.argv)

    def compare(cur, exp):
        def remove_time_fields(res):
            allowed_fields = ["cost", "educate", "solution", "evolution"]
            res = {k: v for k,v in res.items() if k in allowed_fields}
            res["evolution"] = [
                {k: v for k,v in evol.items() if k != "time"} 
                for evol in res["evolution"]
            ]
            return res
            
        with open(cur, "r") as f:
            cur_data = remove_time_fields(json.loads(f.read()))
        with open(exp, "r") as f:
            exp_data = remove_time_fields(json.loads(f.read()))

        if cur_data != exp_data:
            with open(cur, "r") as f:
                print(f.read())
            return 1

        return 0

    exit(compare(args.res_current, args.res_expected))
