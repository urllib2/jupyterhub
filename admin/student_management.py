#!/usr/bin/env python3
import sqlite3
import datetime
from typing import Dict, List, Optional


class StudentManager:
    def __init__(self, db_path='/home/sami/ros2-teaching/data/jupyterhub_data/students.db'):
        self.db_path = db_path
        self.init_database()

    def init_database(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS students (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                first_name TEXT NOT NULL,
                last_name TEXT NOT NULL,
                github_username TEXT UNIQUE,
                email TEXT UNIQUE,
                enrollment_date DATE NOT NULL,
                payment_method TEXT NOT NULL DEFAULT 'unknown',
                payment_amount REAL DEFAULT 0.0,
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
                    enrollment_date: str = None,
                    payment_method: str = 'unknown', payment_amount: float = 0.0,
                    notes: str = None) -> bool:
        if not github_username and not email:
            print("❌ Must provide --github or --gmail")
            return False

        enrollment_date = enrollment_date or datetime.date.today().isoformat()
        payment_date = datetime.date.today().isoformat()

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        try:
            cursor.execute('''
                INSERT INTO students (first_name, last_name, github_username, email,
                                      enrollment_date, payment_method, payment_amount,
                                      payment_date, notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (first_name, last_name, github_username, email,
                  enrollment_date, payment_method, payment_amount, payment_date, notes))
            conn.commit()
            identifier = f"github:{github_username}" if github_username else f"google:{email}"
            print(f"✅ Added {first_name} {last_name} ({identifier})")
            return True
        except sqlite3.IntegrityError as e:
            print(f"❌ Student already exists: {e}")
            return False
        finally:
            conn.close()

    def remove_student(self, github_username: str = None, email: str = None) -> bool:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        try:
            if github_username:
                cursor.execute("UPDATE students SET status='inactive', updated_at=? WHERE github_username=? AND status='active'",
                               (datetime.datetime.now().isoformat(), github_username))
                identifier = f"github:{github_username}"
            else:
                cursor.execute("UPDATE students SET status='inactive', updated_at=? WHERE email=? AND status='active'",
                               (datetime.datetime.now().isoformat(), email))
                identifier = f"google:{email}"
            if cursor.rowcount > 0:
                conn.commit()
                print(f"✅ Removed access for {identifier}")
                return True
            else:
                print(f"❌ Student not found or already inactive: {identifier}")
                return False
        finally:
            conn.close()

    def delete_student(self, github_username: str = None, email: str = None) -> bool:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        try:
            if github_username:
                cursor.execute("DELETE FROM students WHERE github_username=?", (github_username,))
                identifier = f"github:{github_username}"
            else:
                cursor.execute("DELETE FROM students WHERE email=?", (email,))
                identifier = f"google:{email}"
            if cursor.rowcount > 0:
                conn.commit()
                print(f"✅ Permanently deleted {identifier}")
                return True
            else:
                print(f"❌ Student not found: {identifier}")
                return False
        finally:
            conn.close()

    def list_students(self, status: str = 'all') -> List[Dict]:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        if status == 'all':
            cursor.execute('SELECT first_name, last_name, github_username, email, enrollment_date, payment_method, payment_amount, status FROM students ORDER BY enrollment_date DESC')
        else:
            cursor.execute('SELECT first_name, last_name, github_username, email, enrollment_date, payment_method, payment_amount, status FROM students WHERE status=? ORDER BY enrollment_date DESC', (status,))
        students = []
        for row in cursor.fetchall():
            students.append({
                'first_name': row[0], 'last_name': row[1],
                'github_username': row[2], 'email': row[3],
                'enrollment_date': row[4], 'payment_method': row[5],
                'payment_amount': row[6], 'status': row[7]
            })
        conn.close()
        return students

    def get_statistics(self) -> Dict:
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('SELECT COUNT(*) FROM students')
        total = cursor.fetchone()[0]
        cursor.execute('SELECT COUNT(*) FROM students WHERE status="active"')
        active = cursor.fetchone()[0]
        cursor.execute('SELECT payment_method, COUNT(*), SUM(payment_amount) FROM students GROUP BY payment_method')
        payment_stats = {row[0]: {'count': row[1], 'total_amount': row[2] or 0} for row in cursor.fetchall()}
        conn.close()
        return {'total_students': total, 'active_students': active, 'payment_breakdown': payment_stats}
