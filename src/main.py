from file_utility import load_data, load_graph


def main():
    data = load_data()
    print(data)

    graph = load_graph()
    print(graph)


if __name__ == "__main__":
    main()
