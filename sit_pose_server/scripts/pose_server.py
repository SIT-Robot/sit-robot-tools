import sqlite3

from typing import NamedTuple, List


class Pose(NamedTuple):
    name: str
    pos_x: float
    pos_y: float
    pos_z: float
    ori_x: float
    ori_y: float
    ori_z: float
    ori_w: float

    def as_dict(self):
        return {
            'name': self.name,
            'position': [self.pos_x, self.pos_y, self.pos_z],
            'orientation': [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
        }

    def as_str(self):
        return str(self.as_dict())


import sqlite3


class PoseServer:
    def __init__(self,
                 database_file: str,
                 namespace: str):
        self.namespace = namespace
        self.conn = sqlite3.connect(database_file)
        self.cur = self.conn.cursor()
        self.cur.execute("""
            CREATE TABLE IF NOT EXISTS poses(
            	name	TEXT PRIMARY KEY,
            	namespace	TEXT NOT NULL,
            	pos_x	REAL NOT NULL,
            	pos_y	REAL NOT NULL,
            	pos_z	REAL NOT NULL,
            	ori_x	REAL NOT NULL,
            	ori_y	REAL NOT NULL,
            	ori_z	REAL NOT NULL,
            	ori_w	REAL NOT NULL
            );
            """)

    def insert(self, pose: Pose):
        self.cur.execute('REPLACE INTO poses VALUES(?,?,?,?,?,?,?,?,?)',
                         (pose.name,
                          self.namespace,
                          pose.pos_x,
                          pose.pos_y,
                          pose.pos_z,
                          pose.ori_x,
                          pose.ori_y,
                          pose.ori_z,
                          pose.ori_w,))
        self.conn.commit()

    def delete(self, name: str):
        self.cur.execute("""
        DELETE 
        FROM poses 
        WHERE name=?
        AND namespace=?""", (name, self.namespace))
        self.conn.commit()

    def has_pose(self, name: str) -> bool:
        self.cur.execute("""
        SELECT * 
        FROM poses 
        WHERE name=?
        AND namespace=?""", (name, self.namespace))
        return len(self.cur.fetchall()) != 0

    def query(self, name: str) -> Pose:
        self.cur.execute("""
        SELECT * 
        FROM poses 
        WHERE name=?
        AND namespace=?""", (name, self.namespace))
        tuples = self.cur.fetchall()[0]
        return Pose(tuples[0], tuples[2], tuples[3], tuples[4],
                    tuples[5], tuples[6], tuples[7], tuples[8])

    def query_all_name(self) -> List[str]:
        self.cur.execute("""
        SELECT name 
        FROM poses 
        WHERE namespace=?""", (self.namespace,))
        return list(map(lambda x: x[0], self.cur.fetchall()))


if __name__ == '__main__':
    pose_server = PoseServer('a.db', '2012')

    po = Pose('as', 0, 1, 0, 0, 0, 0, 0)
    pose_server.insert(po)

    print(pose_server.query('as'))

    pose_server.cur.close()
    pose_server.conn.close()
