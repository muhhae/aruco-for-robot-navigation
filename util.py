from aruco_detector import Object, ObjectType, Direction


def JSONToMarkers(data: list[list[str]]) -> list[Object]:
    markers = []
    for i, row in enumerate(data):
        for j, obj in enumerate(row):
            if obj is None:
                continue
            objtype = ObjectType.ITEM
            if obj[0] == "A":
                objtype = ObjectType.ARUCO_MARKER
            id = None
            if len(obj) > 1:
                id = obj[1:]

            Ti = i - 1
            Tj = j
            Bi = i + 1
            Bj = j
            Ri = i
            Rj = j + 1
            Li = i
            Lj = j - 1

            T = B = R = L = None
            if Ti >= 0:
                T = data[Ti][Tj]
                if T is not None:
                    T = int(T[1:])
            if Bi < len(data):
                B = data[Bi][Bj]
                if B is not None:
                    B = int(B[1:])
            if Rj < len(row):
                R = data[Ri][Rj]
                if R is not None:
                    R = int(R[1:])
            if Lj >= 0:
                L = data[Li][Lj]
                if L is not None:
                    L = int(L[1:])
            markers.append(
                Object(
                    id,
                    objtype,
                    {
                        Direction.T: T,
                        Direction.B: B,
                        Direction.L: L,
                        Direction.R: R,
                    },
                )
            )
    return markers


def main():
    map = [
        [None, "A6", None],
        [None, "A5", None],
        ["A4", "A2", "A3"],
        [None, "A1", None],
        [None, "A0", None],
    ]

    markers = JSONToMarkers(map)
    for m in markers:
        print("id:", m.id)
        print("type:", m.Type)
        print("neighbour", m.neighbour)


if __name__ == "__main__":
    main()
