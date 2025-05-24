#!/usr/bin/env python3

import xmlrpc.client

def list_method_signatures(server_url):
    try:
        server = xmlrpc.client.ServerProxy(server_url)
        methods = server.system.listMethods()
        print("XML-RPC Method Signatures:")
        for method in sorted(methods):
            try:
                signatures = server.system.methodSignature(method)
                print(f"\nMethod: {method}")
                for sig in signatures:
                    return_type, *param_types = sig
                    print(f"  Signature: {return_type} {method}({', '.join(param_types)})")
            except xmlrpc.client.Fault as e:
                print(f"  Error: No signature available for {method} ({e})")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    server_url = "http://192.168.137.101:41414/"
    list_method_signatures(server_url)