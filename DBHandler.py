import os

import sqlite3
from typing import List, Dict, Any, Optional
from datetime import datetime


class InclinoDBHandler:
    """INCLINO SQLite 데이터베이스 핸들러 클래스"""
    
    def __init__(self, db_name: str = "INCLINO.db"):
        """
        데이터베이스 초기화
        
        Args:
            db_name: 데이터베이스 파일명 (기본값: "INCLINO.db")
        """
        self.db_name = db_name
        self.db_path = os.path.join(os.getcwd(), db_name)
        self._initialize_database()
    
    def _initialize_database(self):
        """데이터베이스 및 테이블 초기화"""
        try:
            # 데이터베이스 연결 (없으면 생성)
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # tblInitVal 테이블 생성
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS tblInitVal (
                    val_roll TEXT,
                    val_pitch TEXT,
                    check_time TEXT
                )
            """)
            
            # tblSensorData 테이블 생성
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS tblSensorData (
                    Val1 TEXT,
                    Val2 TEXT,
                    Val3 TEXT,
                    Val4 TEXT,
                    Val5 TEXT,
                    Val6 TEXT,
                    Val7 TEXT,
                    Val8 TEXT,
                    Val9 TEXT,
                    Val10 TEXT,
                    Val11 TEXT,
                    Val12 TEXT,
                    Val13 TEXT,
                    Val14 TEXT,
                    Val15 TEXT,
                    check_time TEXT
                )
            """)
            
            conn.commit()
            conn.close()
            print(f"데이터베이스 '{self.db_name}' 초기화 완료")
            
        except sqlite3.Error as e:
            print(f"데이터베이스 초기화 중 오류 발생: {e}")
    
    def _get_connection(self) -> sqlite3.Connection:
        """데이터베이스 연결 반환"""
        return sqlite3.connect(self.db_path)
    
    # ==================== tblInitVal CRUD ====================
    
    def insert_init_val(self, val_roll: str, val_pitch: str, check_time: str = None) -> bool:
        """
        tblInitVal에 데이터 삽입
        
        Args:
            val_roll: 롤 값
            val_pitch: 피치 값
            check_time: 체크 시간 (None이면 현재 시간)
            
        Returns:
            bool: 삽입 성공 여부
        """
        try:
            if check_time is None:
                check_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute("""
                INSERT INTO tblInitVal (val_roll, val_pitch, check_time)
                VALUES (?, ?, ?)
            """, (val_roll, val_pitch, check_time))
            
            conn.commit()
            conn.close()
            return True
            
        except sqlite3.Error as e:
            print(f"tblInitVal 삽입 중 오류 발생: {e}")
            return False
    
    def select_init_val(self, limit: int = None) -> List[Dict[str, Any]]:
        """
        tblInitVal 데이터 조회
        
        Args:
            limit: 조회할 레코드 수 제한
            
        Returns:
            List[Dict]: 조회된 데이터 리스트
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            query = "SELECT val_roll, val_pitch, check_time FROM tblInitVal"
            if limit:
                query += f" LIMIT {limit}"
            
            cursor.execute(query)
            rows = cursor.fetchall()
            
            result = []
            for row in rows:
                result.append({
                    'val_roll': row[0],
                    'val_pitch': row[1],
                    'check_time': row[2]
                })
            
            conn.close()
            return result
            
        except sqlite3.Error as e:
            print(f"tblInitVal 조회 중 오류 발생: {e}")
            return []
    
    def update_init_val(self, val_roll: str, val_pitch: str, check_time: str) -> bool:
        """
        tblInitVal 데이터 업데이트 (check_time 기준)
        
        Args:
            val_roll: 새로운 롤 값
            val_pitch: 새로운 피치 값
            check_time: 업데이트할 레코드의 check_time
            
        Returns:
            bool: 업데이트 성공 여부
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute("""
                UPDATE tblInitVal 
                SET val_roll = ?, val_pitch = ?
                WHERE check_time = ?
            """, (val_roll, val_pitch, check_time))
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblInitVal 업데이트 중 오류 발생: {e}")
            return False
    
    def delete_init_val(self, check_time: str = None) -> bool:
        """
        tblInitVal 데이터 삭제
        
        Args:
            check_time: 삭제할 레코드의 check_time (None이면 모든 레코드 삭제)
            
        Returns:
            bool: 삭제 성공 여부
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            if check_time:
                cursor.execute("DELETE FROM tblInitVal WHERE check_time = ?", (check_time,))
            else:
                cursor.execute("DELETE FROM tblInitVal")
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblInitVal 삭제 중 오류 발생: {e}")
            return False
    
    # ==================== tblSensorData CRUD ====================
    
    def insert_sensor_data(self, check_time: str = None, **kwargs) -> bool:
        """
        tblSensorData에 데이터 삽입
        
        Args:
            check_time: 체크 시간 (None이면 현재 시간)
            **kwargs: Val1~Val15 컬럼 값들
            
        Returns:
            bool: 삽입 성공 여부
        """
        try:
            if check_time is None:
                check_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            conn = self._get_connection()
            cursor = conn.cursor()
            
            # 컬럼명과 값 준비
            columns = [f"Val{i}" for i in range(1, 16)]
            columns.append("check_time")
            
            values = [kwargs.get(f"Val{i}", '') for i in range(1, 16)]
            values.append(check_time)
            
            placeholders = ', '.join(['?' for _ in columns])
            column_names = ', '.join(columns)
            
            cursor.execute(f"""
                INSERT INTO tblSensorData ({column_names})
                VALUES ({placeholders})
            """, values)
            
            conn.commit()
            conn.close()
            return True
            
        except sqlite3.Error as e:
            print(f"tblSensorData 삽입 중 오류 발생: {e}")
            return False
    
    def select_sensor_data(self, limit: int = None) -> List[Dict[str, Any]]:
        """
        tblSensorData 데이터 조회
        
        Args:
            limit: 조회할 레코드 수 제한
            
        Returns:
            List[Dict]: 조회된 데이터 리스트
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            query = "SELECT * FROM tblSensorData"
            if limit:
                query += f" LIMIT {limit}"
            
            cursor.execute(query)
            rows = cursor.fetchall()
            
            result = []
            for row in rows:
                data = {}
                for i in range(15):
                    data[f'Val{i+1}'] = row[i]
                data['check_time'] = row[15]  # check_time 컬럼 추가
                result.append(data)
            
            conn.close()
            return result
            
        except sqlite3.Error as e:
            print(f"tblSensorData 조회 중 오류 발생: {e}")
            return []
    
    def update_sensor_data(self, check_time: str = None, row_id: int = None, **kwargs) -> bool:
        """
        tblSensorData 데이터 업데이트 (check_time 또는 ROWID 기준)
        
        Args:
            check_time: 업데이트할 레코드의 check_time (우선순위)
            row_id: 업데이트할 레코드의 ROWID (check_time이 None일 때 사용)
            **kwargs: 업데이트할 컬럼 값들
            
        Returns:
            bool: 업데이트 성공 여부
        """
        try:
            if not check_time and not row_id:
                print("check_time 또는 row_id 중 하나는 필수입니다.")
                return False
            
            conn = self._get_connection()
            cursor = conn.cursor()
            
            # 업데이트할 컬럼들 준비
            set_clauses = []
            values = []
            
            for key, value in kwargs.items():
                if key.startswith('Val') and key[3:].isdigit():
                    set_clauses.append(f"{key} = ?")
                    values.append(value)
                elif key == 'check_time':
                    set_clauses.append(f"{key} = ?")
                    values.append(value)
            
            if not set_clauses:
                return False
            
            # WHERE 조건 설정
            if check_time:
                where_clause = "check_time = ?"
                values.append(check_time)
            else:
                where_clause = "ROWID = ?"
                values.append(row_id)
            
            cursor.execute(f"""
                UPDATE tblSensorData 
                SET {', '.join(set_clauses)}
                WHERE {where_clause}
            """, values)
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblSensorData 업데이트 중 오류 발생: {e}")
            return False
    
    def delete_sensor_data(self, check_time: str = None, row_id: int = None) -> bool:
        """
        tblSensorData 데이터 삭제
        
        Args:
            check_time: 삭제할 레코드의 check_time (우선순위)
            row_id: 삭제할 레코드의 ROWID (check_time이 None일 때 사용)
            둘 다 None이면 모든 레코드 삭제
            
        Returns:
            bool: 삭제 성공 여부
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            if check_time:
                cursor.execute("DELETE FROM tblSensorData WHERE check_time = ?", (check_time,))
            elif row_id:
                cursor.execute("DELETE FROM tblSensorData WHERE ROWID = ?", (row_id,))
            else:
                cursor.execute("DELETE FROM tblSensorData")
            
            conn.commit()
            affected_rows = cursor.rowcount
            conn.close()
            
            return affected_rows > 0
            
        except sqlite3.Error as e:
            print(f"tblSensorData 삭제 중 오류 발생: {e}")
            return False
    
    # ==================== 유틸리티 메서드 ====================
    
    def get_table_info(self, table_name: str) -> List[Dict[str, Any]]:
        """
        테이블 정보 조회
        
        Args:
            table_name: 테이블명
            
        Returns:
            List[Dict]: 테이블 구조 정보
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute(f"PRAGMA table_info({table_name})")
            rows = cursor.fetchall()
            
            result = []
            for row in rows:
                result.append({
                    'cid': row[0],
                    'name': row[1],
                    'type': row[2],
                    'notnull': row[3],
                    'dflt_value': row[4],
                    'pk': row[5]
                })
            
            conn.close()
            return result
            
        except sqlite3.Error as e:
            print(f"테이블 정보 조회 중 오류 발생: {e}")
            return []
    
    def get_row_count(self, table_name: str) -> int:
        """
        테이블의 레코드 수 조회
        
        Args:
            table_name: 테이블명
            
        Returns:
            int: 레코드 수
        """
        try:
            conn = self._get_connection()
            cursor = conn.cursor()
            
            cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
            count = cursor.fetchone()[0]
            
            conn.close()
            return count
            
        except sqlite3.Error as e:
            print(f"레코드 수 조회 중 오류 발생: {e}")
            return 0
    
    def close(self):
        """데이터베이스 연결 종료 (현재 구현에서는 각 메서드마다 연결을 닫으므로 placeholder)"""
        pass


# 사용 예제
if __name__ == "__main__":
    # 데이터베이스 핸들러 생성
    db = InclinoDBHandler()
    
    # tblInitVal 테스트
    print("=== tblInitVal 테스트 ===")
    
    # 데이터 삽입
    db.insert_init_val("10.5", "20.3")
    db.insert_init_val("11.2", "21.1")
    
    # 데이터 조회
    init_data = db.select_init_val()
    print(f"tblInitVal 레코드 수: {len(init_data)}")
    for data in init_data:
        print(f"Roll: {data['val_roll']}, Pitch: {data['val_pitch']}, Time: {data['check_time']}")
    
    # tblSensorData 테스트
    print("\n=== tblSensorData 테스트 ===")
    
    # 데이터 삽입
    db.insert_sensor_data(Val1="1.1", Val2="2.2", Val3="3.3", Val4="4.4", Val5="5.5")
    db.insert_sensor_data(Val1="1.2", Val2="2.3", Val3="3.4", Val4="4.5", Val5="5.6")
    
    # 데이터 조회
    sensor_data = db.select_sensor_data()
    print(f"tblSensorData 레코드 수: {len(sensor_data)}")
    for data in sensor_data:
        print(f"Val1: {data['Val1']}, Val2: {data['Val2']}, Val3: {data['Val3']}, Time: {data['check_time']}")
    
    # 테이블 정보 조회
    print("\n=== 테이블 정보 ===")
    print(f"tblInitVal 레코드 수: {db.get_row_count('tblInitVal')}")
    print(f"tblSensorData 레코드 수: {db.get_row_count('tblSensorData')}")