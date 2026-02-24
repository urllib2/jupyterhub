#!/usr/bin/env python3
import sqlite3
import datetime
from typing import Dict, List, Optional

class StudentManager:
    def __init__(self, db_path='/srv/jupyterhub/data/students.db'):
        self.db_path = db_path
        self.init_database()

    def init_database(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS students (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                github_username TEXT UNIQUE,
                email TEXT UNIQUE,
                first_name TEXT NOT NULL,
                last_name TEXT NOT NULL,
                enrollment_date DATE NOT NULL,
                payment_method TEXT NOT NULL,
                payment_amount REAL,
                payment_date DATE,
                status TEXT DEFAULT 'active',
                notes TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        conn.commit()
        conn.close()

    def add_student(self, first_name: str, last_name: str,
                    github_username: str = None, email: str = None,
                    payment_method: str = 'unknown', payment_amount: float = 0.0,
                    payment_date: str = None, enrollment_date: str = None,
                    notes: str = None) -> bool:

        if not github_username and not email:
            print("❌ Must provide at least github_username or email")
            return False

        enrollment_date = enrollment_date or datetime.date.today().isoformat()
        payment_date = payment_date or datetime.date.today().isoformat()

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        try:
            cursor.execute('''
                INSERT INTO students (github_username, email, first_name, last_name,
                                     enrollment_date, payment_method, payment_amount,
                                     payment_date, notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (github_username, email, first_name, last_name,
                  enrollment_date, payment_method, payment_amount,
                  payment_date, notes))
            conn.commit()
            print(f"✅ Added {first_name} {last_name}")
            print(f"   GitHub: {github_username or '-'}")
            print(f"   Email:  {email or '-'}")
            print(f"   Enrolled: {enrollment_date} | Payment: {payment_method} - {payment_amount} TND")
            return True
        except sqlite3.IntegrityError as e:
            print(f"❌ Student already exists: {e}")
            return False
        finally:
            conn.close()

    def remove_student(self, identifier: str) -> bool:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        try:
            cursor.execute('''
                UPDATE students SET status = 'inactive', updated_at = ?
                WHERE (github_username = ? OR email = ?) AND status = 'active'
            ''', (datetime.datetime.now().isoformat(), identifier, identifier))
            if cursor.rowcount > 0:
                conn.commit()
                print(f"✅ Removed access for {identifier}")
                return True
            else:
                print(f"❌ Student not found or already inactive")
                return False
        finally:
            conn.close()

    def get_active_students(self) -> List[Dict]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT github_username, email FROM students WHERE status = 'active'")
        students = [{'github': row[0], 'email': row[1]} for row in cursor.fetchall()]
        conn.close()
        return students

    def list_students(self, status: str = 'all') -> List[Dict]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        if status == 'all':
            cursor.execute('''SELECT github_username, email, first_name, last_name,
                              enrollment_date, payment_method, payment_amount, status, notes
                              FROM students ORDER BY enrollment_date DESC''')
        else:
            cursor.execute('''SELECT github_username, email, first_name, last_name,
                              enrollment_date, payment_method, payment_amount, status, notes
                              FROM students WHERE status = ? ORDER BY enrollment_date DESC''', (status,))
        students = []
        for row in cursor.fetchall():
            students.append({
                'github_username': row[0],
                'email': row[1],
                'first_name': row[2],
                'last_name': row[3],
                'enrollment_date': row[4],
                'payment_method': row[5],
                'payment_amount': row[6],
                'status': row[7],
                'notes': row[8]
            })
        conn.close()
        return students

    def get_statistics(self) -> Dict:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('SELECT COUNT(*) FROM students')
        total = cursor.fetchone()[0]
        cursor.execute('SELECT COUNT(*) FROM students WHERE status = "active"')
        active = cursor.fetchone()[0]
        cursor.execute('''SELECT payment_method, COUNT(*), SUM(payment_amount)
                          FROM students GROUP BY payment_method''')
        payment_stats = {}
        for row in cursor.fetchall():
            payment_stats[row[0]] = {'count': row[1], 'total_amount': row[2] or 0}
        thirty_days_ago = (datetime.date.today() - datetime.timedelta(days=30)).isoformat()
        cursor.execute('SELECT COUNT(*) FROM students WHERE enrollment_date >= ?', (thirty_days_ago,))
        recent = cursor.fetchone()[0]
        conn.close()
        return {
            'total_students': total,
            'active_students': active,
            'payment_breakdown': payment_stats,
            'recent_enrollments_30d': recent
        }
