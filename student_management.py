#!/usr/bin/env python3
import sqlite3
import json
import datetime
from typing import Dict, List, Optional
import csv

class StudentManager:
    def __init__(self, db_path='/srv/jupyterhub/data/students.db'):
        self.db_path = db_path
        self.init_database()
    
    def init_database(self):
        """Initialize SQLite database with proper schema"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS students (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                github_username TEXT UNIQUE NOT NULL,
                first_name TEXT NOT NULL,
                last_name TEXT NOT NULL,
                email TEXT,
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
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS cohorts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT UNIQUE NOT NULL,
                start_date DATE NOT NULL,
                end_date DATE NOT NULL,
                max_students INTEGER,
                status TEXT DEFAULT 'active',
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        conn.commit()
        conn.close()
    
    def add_student(self, github_username: str, first_name: str, last_name: str, 
                   payment_method: str = 'unknown', payment_amount: float = 0.0,
                   email: str = None, payment_date: str = None, 
                   enrollment_date: str = None, notes: str = None) -> bool:
        """Add a new student when they pay"""
        
        enrollment_date = enrollment_date or datetime.date.today().isoformat()
        payment_date = payment_date or datetime.date.today().isoformat()
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            cursor.execute('''
                INSERT INTO students (github_username, first_name, last_name, 
                                    email, enrollment_date, payment_method, 
                                    payment_amount, payment_date, notes)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (github_username, first_name, last_name, email, 
                  enrollment_date, payment_method, payment_amount, 
                  payment_date, notes))
            
            conn.commit()
            print(f"✅ Added {first_name} {last_name} ({github_username})")
            print(f"   Enrolled: {enrollment_date}")
            print(f"   Payment: {payment_method} - ${payment_amount}")
            return True
            
        except sqlite3.IntegrityError:
            print(f"❌ Student {github_username} already exists!")
            return False
        finally:
            conn.close()
    
    def remove_student(self, github_username: str) -> bool:
        """Remove student access (keep record, mark inactive)"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            cursor.execute('''
                UPDATE students SET status = 'inactive', updated_at = ?
                WHERE github_username = ? AND status = 'active'
            ''', (datetime.datetime.now().isoformat(), github_username))
            
            if cursor.rowcount > 0:
                conn.commit()
                print(f"✅ Removed access for {github_username}")
                return True
            else:
                print(f"❌ Student {github_username} not found or already inactive")
                return False
        finally:
            conn.close()
    
    def get_active_students(self) -> List[str]:
        """Get list of active students for JupyterHub"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT github_username FROM students 
            WHERE status = 'active' 
            ORDER BY enrollment_date
        ''')
        
        students = [row[0] for row in cursor.fetchall()]
        conn.close()
        return students
    
    def list_students(self, status: str = 'all') -> List[Dict]:
        """List students with full information"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        if status == 'all':
            cursor.execute('''
                SELECT github_username, first_name, last_name, 
                       enrollment_date, payment_method, payment_amount,
                       status, notes
                FROM students 
                ORDER BY enrollment_date DESC
            ''')
        else:
            cursor.execute('''
                SELECT github_username, first_name, last_name, 
                       enrollment_date, payment_method, payment_amount,
                       status, notes
                FROM students 
                WHERE status = ?
                ORDER BY enrollment_date DESC
            ''', (status,))
        
        students = []
        for row in cursor.fetchall():
            students.append({
                'github_username': row[0],
                'first_name': row[1],
                'last_name': row[2],
                'enrollment_date': row[3],
                'payment_method': row[4],
                'payment_amount': row[5],
                'status': row[6],
                'notes': row[7]
            })
        
        conn.close()
        return students
    
    def get_student_info(self, github_username: str) -> Optional[Dict]:
        """Get detailed information about a specific student"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT * FROM students 
            WHERE github_username = ?
        ''', (github_username,))
        
        row = cursor.fetchone()
        conn.close()
        
        if row:
            return {
                'id': row[0],
                'github_username': row[1],
                'first_name': row[2],
                'last_name': row[3],
                'email': row[4],
                'enrollment_date': row[5],
                'payment_method': row[6],
                'payment_amount': row[7],
                'payment_date': row[8],
                'status': row[9],
                'notes': row[10],
                'created_at': row[11],
                'updated_at': row[12]
            }
        return None
    
    def get_statistics(self) -> Dict:
        """Get cohort statistics"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Total students
        cursor.execute('SELECT COUNT(*) FROM students')
        total = cursor.fetchone()[0]
        
        # Active students
        cursor.execute('SELECT COUNT(*) FROM students WHERE status = "active"')
        active = cursor.fetchone()[0]
        
        # Payment method breakdown
        cursor.execute('''
            SELECT payment_method, COUNT(*), SUM(payment_amount)
            FROM students
            GROUP BY payment_method
        ''')
        
        payment_stats = {}
        for row in cursor.fetchall():
            payment_stats[row[0]] = {
                'count': row[1],
                'total_amount': row[2] or 0
            }
        
        # Recent enrollments (last 30 days)
        thirty_days_ago = (datetime.date.today() - datetime.timedelta(days=30)).isoformat()
        cursor.execute('''
            SELECT COUNT(*) FROM students 
            WHERE enrollment_date >= ?
        ''', (thirty_days_ago,))
        
        recent_enrollments = cursor.fetchone()[0]
        
        conn.close()
        
        return {
            'total_students': total,
            'active_students': active,
            'payment_breakdown': payment_stats,
            'recent_enrollments_30d': recent_enrollments
        }
