from py4j.java_gateway import JavaGateway


def main():
    gateway = JavaGateway()
    epos_server = gateway.entry_point
    print(epos_server.Message())


if __name__ == "__main__":
    main()
