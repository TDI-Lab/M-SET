from py4j.java_gateway import JavaGateway


def main():
    gateway = JavaGateway()
    random = gateway.jvm.java.util.Random()
    number1 = random.nextInt(10)
    number2 = random.nextInt(10)
    print(number1, number2)


if __name__ == "__main__":
    main()
