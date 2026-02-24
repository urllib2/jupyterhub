#!/usr/bin/env python3
import sys
import argparse
from student_management import StudentManager

DB_DEFAULT = '/home/sami/ros2-teaching/data/jupyterhub_data/students.db'

def main():
    parser = argparse.ArgumentParser(description='ROSForge Student Management')
    parser.add_argument('--db', default=DB_DEFAULT, help='Path to students.db')
    subparsers = parser.add_subparsers(dest='command', help='Commands')
    
    # Add student
    add_parser = subparsers.add_parser('add', help='Add new student')
    add_parser.add_argument('first_name', help='First name')
    add_parser.add_argument('last_name', help='Last name')
    add_parser.add_argument('--github', help='GitHub username (optional)')
    add_parser.add_argument('--email', help='Gmail address (optional)')
    add_parser.add_argument('--payment', default='unknown', choices=['cash', 'bank', 'transfer', 'other'], help='Payment method')
    add_parser.add_argument('--amount', type=float, default=0.0, help='Payment amount')
    add_parser.add_argument('--notes', help='Additional notes')

    # Remove student
    remove_parser = subparsers.add_parser('remove', help='Remove student access')
    remove_parser.add_argument('identifier', help='GitHub username or Gmail address')

    # List students
    list_parser = subparsers.add_parser('list', help='List students')
    list_parser.add_argument('--status', choices=['active', 'inactive', 'all'], default='all', help='Filter by status')

    # Statistics
    subparsers.add_parser('stats', help='Show statistics')

    args = parser.parse_args()
    manager = StudentManager(db_path=args.db)

    if args.command == 'add':
        if not args.github and not args.email:
            print("❌ You must provide at least --github or --email")
            sys.exit(1)

        success = manager.add_student(
            first_name=args.first_name,
            last_name=args.last_name,
            github_username=args.github,
            email=args.email,
            payment_method=args.payment,
            payment_amount=args.amount,
            notes=args.notes
        )
        if success:
            print("✅ Student added successfully!")
        else:
            print("❌ Failed to add student")
            sys.exit(1)

    elif args.command == 'remove':
        success = manager.remove_student(args.identifier)
        if success:
            print("✅ Student access removed!")
        else:
            print("❌ Student not found")
            sys.exit(1)

    elif args.command == 'list':
        students = manager.list_students(args.status)
        if students:
            print(f"📋 {args.status.title()} Students:")
            print(f"{'Name':<25} {'GitHub':<20} {'Email':<30} {'Enrolled':<12} {'Amount':<8}")
            print("=" * 100)
            for s in students:
                name = f"{s['first_name']} {s['last_name']}"
                print(f"{name:<25} {(s['github_username'] or '-'):<20} {(s['email'] or '-'):<30} {s['enrollment_date']:<12} {s['payment_amount']:<8}")
        else:
            print("No students found.")

    elif args.command == 'stats':
        stats = manager.get_statistics()
        print("📊 Cohort Statistics:")
        print(f"Total students: {stats['total_students']}")
        print(f"Active students: {stats['active_students']}")
        print(f"Recent enrollments (30d): {stats['recent_enrollments_30d']}")
        print("\nPayment Methods:")
        for method, data in stats['payment_breakdown'].items():
            print(f"  {method}: {data['count']} students, {data['total_amount']:.2f} TND total")
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
